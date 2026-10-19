#!/usr/bin/env python3
"""
debug_fisker_secoc.py — SecOC MAC / freshness-counter forensics for the Ocean lateral abort.

Runs ALONGSIDE sunnypilot (read-only: subscribes to cereal 'can' + 'sendcan', touches
nothing in the control path). Run it together with debug_fisker_lateral.py — that one logs
the signal transitions / CMP / TX cadence; THIS one answers the one question those can't:

    does our SecOC MAC match what the EPS actually accepts?

The decisive test — OEM-as-ground-truth
----------------------------------------
The OEM ADAS keeps transmitting ADAS_0x1D0 / ADAS_0x1C0 on bus 2 the whole time (we only
block them from reaching the EPS). Those frames carry a MAC the EPS *accepts*. So for every
OEM bus-2 frame we recompute the MAC locally with our stored SecOCKey + our per-window
counter reconstruction and compare to the MAC OEM put on the wire:

  * MATCH    -> our key + algorithm + full-counter reconstruction are all correct. A clean
               mid-window takeover SHOULD therefore be accepted, so the abort is a handover
               offset / timing problem, not the MAC math.
  * MISMATCH -> our reconstruction (or algorithm) is wrong — exactly why the EPS rejects our
               injected frames. We then BRUTE-FORCE the msg_counter (and, if needed, the
               reset/trip) that reproduces OEM's MAC, which tells us the value the EPS really
               expects and therefore the fix.

You don't even have to engage to get this: just run and drive straight. If the OEM MAC line
says MISMATCH, we've found it. If it says MATCH, engage, let it abort, and the ring-buffer
dump (printed the instant EPS_AbortFb goes non-zero) shows the exact per-frame picture —
our TX vs OEM, wire + full counter + MAC verify — around the reject.

Usage (on the comma 3X):
    scp scripts/debug_fisker_secoc.py /data/openpilot/scripts/
    cd /data/openpilot
    PYTHONPATH=. python3 scripts/debug_fisker_secoc.py

Writes /data/fisker_secoc_debug.log (append) AND stdout. Ctrl-C to stop.
"""

from __future__ import annotations

import datetime
import time
from collections import deque
from pathlib import Path

from openpilot.cereal import messaging
from opendbc.can.parser import CANParser
from opendbc.car.fisker.secoc import secoc_mac, verify_sync, parse_sync, SYNC_CAN_ID

ADAS_DBC = "fisker_ocean_adas"
ADDR_1D0 = 0x1D0
ADDR_1C0 = 0x1C0
ADDR_SYNC = 0x20   # == SYNC_CAN_ID (GW_Syn_All)
OEM_BUS = 2        # OEM ADAS's own frames (ground truth the EPS accepts)
OUR_BUS = 0        # ADASBUS — where our injected frames go out

# How wide to brute-force before giving up. 1<<16 covers the whole 16-bit msg_counter.
BRUTE_LIMIT = 1 << 16
MAX_BRUTES = 4     # cap the (slow) brute-force to the first few mismatches


def fmt_ts() -> str:
  return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def hexb(b: bytes | None) -> str:
  return b.hex() if b else "--"


class FullCtr:
  """Reconstruct the full per-window SecOC msg_counter from the 6-bit on-wire value,
  mirroring carstate: the high bits are a PER-GW-RESET-WINDOW count — zero them at each
  reset, +1 on every 63->0 wrap within the window."""
  def __init__(self):
    self.reset_tracked = None
    self.wire_last = None
    self.wraps = 0

  def update(self, wire: int, reset: int) -> int:
    if reset != self.reset_tracked:
      self.reset_tracked = reset
      self.wraps = 0
    elif self.wire_last is not None and wire < self.wire_last:
      self.wraps += 1
    self.wire_last = wire
    return self.wraps * 64 + wire


def search_freshness(key, can_id, pdu, mac_wire, trip, reset, full_guess):
  """Find the (msg_counter[, reset][, trip]) that reproduces mac_wire. Returns a dict
  describing what matched, or None. Layered so the common case (counter offset) is fast."""
  # 1) narrow window around our reconstruction — catches a small high-bit / wrap offset
  lo = max(0, full_guess - 512)
  for m in range(lo, full_guess + 512):
    if secoc_mac(key, can_id, pdu, trip, reset, m) == mac_wire:
      return {"kind": "msg_counter", "msg": m, "trip": trip, "reset": reset}
  # 2) full 16-bit msg_counter sweep
  for m in range(BRUTE_LIMIT):
    if secoc_mac(key, can_id, pdu, trip, reset, m) == mac_wire:
      return {"kind": "msg_counter", "msg": m, "trip": trip, "reset": reset}
  # 3) maybe our reset is off by a window
  for dr in (1, -1, 2, -2, 3, -3):
    r = reset + dr
    for m in range(BRUTE_LIMIT):
      if secoc_mac(key, can_id, pdu, trip, r, m) == mac_wire:
        return {"kind": "reset_off", "msg": m, "trip": trip, "reset": r, "reset_delta": dr}
  # 4) maybe trip is off
  for dt in (1, -1):
    t = trip + dt
    for m in range(BRUTE_LIMIT):
      if secoc_mac(key, can_id, pdu, t, reset, m) == mac_wire:
        return {"kind": "trip_off", "msg": m, "trip": t, "reset": reset, "trip_delta": dt}
  return None


def main():
  log_path = Path("/data/fisker_secoc_debug.log")
  log_f = open(log_path, "a", buffering=1)

  def say(s: str, *, loud: bool = False) -> None:
    line = f"[{fmt_ts()}] {'***' if loud else '   '} {s}"
    print(line, flush=True)
    log_f.write(line + "\n")

  say("=== fisker SecOC forensics ===", loud=True)

  # ---- load the key exactly as card.py does: hex string in params 'SecOCKey' ----
  key = None
  try:
    from openpilot.common.params import Params
    raw = Params().get("SecOCKey")
    if raw is not None:
      txt = raw.decode() if isinstance(raw, (bytes, bytearray)) else raw
      key = bytes.fromhex(txt.strip())
  except Exception as e:  # noqa: BLE001
    say(f"ERROR reading SecOCKey from params: {e}", loud=True)
  if not key or len(key) != 16:
    say(f"ERROR: need a 16-byte SecOCKey in params (got {len(key) if key else 0} bytes). Cannot verify MACs.", loud=True)
    return
  say(f"SecOCKey loaded ({len(key)} bytes).")

  can_sock = messaging.sub_sock("can", conflate=False, timeout=50)
  sendcan_sock = messaging.sub_sock("sendcan", conflate=False, timeout=50)
  # EPS reply (0x1C2) via CANParser — gives us AdasLatCtrlSts / AbortFb to trigger the dump.
  cp_eps = CANParser(ADAS_DBC, [("EPS_0x1C2", 0)], 0)

  trip = 0
  reset = 0
  sync_seen = False
  sync_ok = None

  trackers = {
    ("oem", ADDR_1D0): FullCtr(), ("oem", ADDR_1C0): FullCtr(),
    ("our", ADDR_1D0): FullCtr(), ("our", ADDR_1C0): FullCtr(),
  }
  stats = {k: [0, 0] for k in trackers}  # [match, mismatch]
  brutes_done = 0
  first_mismatch_logged = set()

  ring = deque(maxlen=120)  # ~1.2 s of lateral frames for the abort dump
  last_sts = None
  last_abort = 0
  last_summary = time.monotonic()

  def verify_frame(tag: str, addr: int, data: bytes, log_mono: int):
    """Recompute the SecOC MAC for one frame and compare to the wire MAC. tag in {oem,our}."""
    nonlocal brutes_done
    if len(data) < 8:
      return
    wire = (data[4] >> 2) & 0x3F
    rflag = data[4] & 0x3
    pdu = bytes(data[0:4])
    mac_wire = bytes(data[5:8])
    full = trackers[(tag, addr)].update(wire, reset)
    mac_calc = secoc_mac(key, addr, pdu, trip, reset, full)
    match = (mac_calc == mac_wire)
    stats[(tag, addr)][0 if match else 1] += 1
    ring.append((log_mono, tag, addr, wire, rflag, full, mac_wire, mac_calc, match))

    # Cross-check: the wire reset-flag must equal reset&0x3 (confirms we're on the right window).
    rflag_ok = (rflag == (reset & 0x3))

    key_name = f"{tag}_{addr:03x}"
    if not match and key_name not in first_mismatch_logged:
      first_mismatch_logged.add(key_name)
      say(f"MISMATCH {tag} 0x{addr:03X}: wire_mac={hexb(mac_wire)} calc_mac={hexb(mac_calc)} "
          f"wire_ctr={wire} full_ctr={full} rflag={rflag}(reset&3={reset & 0x3},ok={rflag_ok}) "
          f"trip={trip} reset={reset}", loud=True)
      if tag == "oem" and brutes_done < MAX_BRUTES:
        brutes_done += 1
        say(f"  brute-forcing the counter OEM really used (pdu={hexb(pdu)})...", loud=True)
        t0 = time.monotonic()
        found = search_freshness(key, addr, pdu, mac_wire, trip, reset, full)
        dt = time.monotonic() - t0
        if found is None:
          say(f"  NO match found in {dt:.1f}s — key/DataId/algorithm likely differ for 0x{addr:03X}", loud=True)
        else:
          off = found["msg"] - full
          say(f"  FOUND in {dt:.1f}s: {found['kind']} msg_counter={found['msg']} "
              f"(ours={full}, offset={off:+d}) trip={found['trip']} reset={found['reset']}", loud=True)
          say(f"  => EPS expects full msg_counter {found['msg']} where we reconstruct {full}. "
              f"{'OUR RECONSTRUCTION IS OFF BY ' + format(off, '+d') if off else 'reset/trip interpretation differs'}.", loud=True)

  def dump_ring(reason: str):
    say(f"---- RING DUMP ({reason}): last {len(ring)} lateral frames (wire/full ctr + MAC verify) ----", loud=True)
    t_now = time.monotonic_ns()
    for (lm, tag, addr, wire, rflag, full, mw, mc, match) in ring:
      age_ms = (t_now - lm) / 1e6
      flag = "OK " if match else "BAD"
      say(f"    -{age_ms:6.1f}ms {tag:3s} 0x{addr:03X} wire={wire:2d} full={full:5d} rflag={rflag} "
          f"mac_wire={hexb(mw)} mac_calc={hexb(mc)} [{flag}]")
    say(f"---- end ring dump ----", loud=True)

  try:
    while True:
      events = []  # (log_mono, 'rx'|'tx', addr, data, src)
      can_batch = []
      for evt in messaging.drain_sock(can_sock, wait_for_one=False):
        for c in evt.can:
          events.append((evt.logMonoTime, "rx", c.address, bytes(c.dat), c.src))
        can_batch.append((evt.logMonoTime, [(c.address, c.dat, c.src) for c in evt.can]))
      for evt in messaging.drain_sock(sendcan_sock, wait_for_one=False):
        for c in evt.sendcan:
          events.append((evt.logMonoTime, "tx", c.address, bytes(c.dat), c.src))

      events.sort(key=lambda e: e[0])
      for (log_mono, direction, addr, data, src) in events:
        # keep trip/reset current in timeline order, parsing 0x20 raw (Trip2 Reset3 MAC3)
        if addr == ADDR_SYNC and len(data) >= 8:
          try:
            trip, reset = parse_sync(bytes(data))
            ok = verify_sync(key, bytes(data))
            if ok != sync_ok:
              say(f"GW sync: trip={trip} reset={reset} key_verify={'OK' if ok else 'FAIL'}", loud=not ok)
            sync_ok, sync_seen = ok, True
          except Exception as e:  # noqa: BLE001
            say(f"sync parse error: {e}")
          continue
        if addr in (ADDR_1D0, ADDR_1C0):
          if direction == "rx" and src == OEM_BUS:
            verify_frame("oem", addr, data, log_mono)
          elif direction == "tx" and src == OUR_BUS:
            verify_frame("our", addr, data, log_mono)

      # EPS status / abort — trigger the ring dump on the reject edge.
      if can_batch:
        cp_eps.update(can_batch)
        sts = int(cp_eps.vl["EPS_0x1C2"]["EPS_AdasLatCtrlSts"])
        abort = int(cp_eps.vl["EPS_0x1C2"]["EPS_AbortFb"])
        if (abort not in (0, 1) and last_abort in (0, 1)) or (sts == 3 and last_sts != 3):
          say(f"EPS REJECT: AdasLatCtrlSts {last_sts}->{sts}  AbortFb {last_abort}->{abort}", loud=True)
          dump_ring(f"abort={abort} sts={sts}")
        elif (sts != last_sts) or (abort != last_abort):
          say(f"EPS: AdasLatCtrlSts {last_sts}->{sts}  AbortFb {last_abort}->{abort}")
        last_sts, last_abort = sts, abort

      now = time.monotonic()
      if now - last_summary > 2.0:
        last_summary = now
        parts = []
        for (tag, addr), (m, mm) in stats.items():
          if m or mm:
            parts.append(f"{tag}0x{addr:03X}:{m}ok/{mm}bad")
        say(f"summary  sync={'OK' if sync_ok else ('FAIL' if sync_seen else '?')} "
            f"trip={trip} reset={reset}  MAC[{'  '.join(parts) if parts else 'no frames yet'}]")

      time.sleep(0.003)

  except KeyboardInterrupt:
    say("=== stopped by user ===", loud=True)
  finally:
    say(f"final MAC stats: " + "  ".join(f"{t}0x{a:03X}:{m}ok/{mm}bad" for (t, a), (m, mm) in stats.items()))
    log_f.close()


if __name__ == "__main__":
  main()
