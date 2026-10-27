#!/usr/bin/env python3
"""
debug_fisker_full.py — comprehensive Fisker SecOC/lateral forensic logger.

Captures EVERY SecOC/lateral-relevant frame on EVERY bus and decodes it, so we can see
the full picture of the OEM->openpilot handover and the candidate substitution:

  source labels:
    oem    = OEM's own 0x1D0 on bus 2 (ground truth the EPS normally follows)
    ourtx  = our direct 0x1D0 on bus 0 (sendcan) — the rolled-back direct approach
    cand   = our candidate 0x5D0 on bus 1 (sendcan) — the panda-substitution approach
    eps    = EPS-bound 0x1D0 echoed by panda as 0x7D0 on bus 0 (ONLY if the debug panda
             build echoes its forwards; otherwise absent — see note)

For each 0x1D0-shaped frame (0x1D0 / 0x5D0 / 0x7D0) it DECODES the true (reset, full
msg_counter) straight from the SecOC MAC with our key (pinning reset via the wire
reset-flag, searching high-bit multiples), plus whether the MAC verifies.

Outputs:
  * /data/fisker_full_raw.log     — raw hex of every watched frame (ts dir bus addr=hex)
  * /data/fisker_full_decode.log  — decoded timeline: SEED CHECK, per-source continuity,
                                     candidate-vs-OEM alignment, EPS state, abort ring dump
Read-only. Run:  cd /data/openpilot && PYTHONPATH=. <venv-python> scripts/debug_fisker_full.py
"""
from __future__ import annotations

import datetime
import time
from collections import deque
from pathlib import Path

from openpilot.cereal import messaging
from opendbc.can.parser import CANParser
from opendbc.car.fisker.secoc import secoc_mac, sync_mac

ADAS_DBC = "fisker_ocean_adas"
A_1D0, A_1C0, A_1C2, A_SYNC, A_CAND, A_ECHO = 0x1D0, 0x1C0, 0x1C2, 0x20, 0x5D0, 0x7D0
WATCH = {A_1D0, A_1C0, A_1C2, A_SYNC, A_CAND, A_ECHO}
OEM_BUS, OUR_BUS, CAND_BUS = 2, 0, 1
MAX_K = 64
RING = 160  # frames of history to dump on an EPS reject


def ts() -> str:
  return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def hx(b) -> str:
  return bytes(b).hex()


def decode_1d0(key, data, trip, gw_reset):
  """True (reset, full_counter, mac_ok) from an 0x1D0-shaped frame's MAC. Returns None if
  we can't reproduce it (undecodable = wrong key/trip/reset/algorithm)."""
  if len(data) < 8:
    return None
  wire = (data[4] >> 2) & 0x3F
  rflag = data[4] & 0x3
  pdu = bytes(data[0:4])
  mac_wire = bytes(data[5:8])
  for r in (gw_reset, gw_reset - 1, gw_reset + 1, gw_reset - 2, gw_reset + 2):
    if r < 0 or (r & 3) != rflag:
      continue
    for k in range(MAX_K):
      m = wire + 64 * k
      if secoc_mac(key, A_1D0, pdu, trip, r, m) == mac_wire:
        return r, m, True
  return wire, rflag, False  # couldn't verify; return wire+rflag for context


def main():
  raw_f = open(Path("/data/fisker_full_raw.log"), "a", buffering=1)
  dec_f = open(Path("/data/fisker_full_decode.log"), "a", buffering=1)

  def say(s, *, loud=False):
    line = f"[{ts()}] {'***' if loud else '   '} {s}"
    print(line, flush=True)
    dec_f.write(line + "\n")

  def raw(line):
    raw_f.write(f"[{ts()}] {line}\n")

  say("=== fisker FULL forensic logger ===", loud=True)

  key = None
  try:
    from openpilot.common.params import Params
    k = Params().get("SecOCKey")
    if k is not None:
      key = bytes.fromhex((k.decode() if isinstance(k, (bytes, bytearray)) else k).strip())
  except Exception as e:  # noqa: BLE001
    say(f"ERROR reading SecOCKey: {e}", loud=True)
  if not key or len(key) != 16:
    say("ERROR: need a 16-byte SecOCKey in params.", loud=True)
    return
  say(f"SecOCKey loaded ({len(key)} bytes). Waiting for GW sync...")

  can_sock = messaging.sub_sock("can", conflate=False, timeout=50)
  sendcan_sock = messaging.sub_sock("sendcan", conflate=False, timeout=50)
  # Mode B (starvation) instrumentation: op-side intent + panda state. conflate=True —
  # we only want the latest each loop. Names vary across sunnypilot forks, so subscribe
  # defensively and tolerate any that don't exist.
  op_socks = {}
  for name in ("carControl", "pandaStates", "selfdriveState", "controlsState", "carState"):
    try:
      op_socks[name] = messaging.sub_sock(name, conflate=True, timeout=10)
    except Exception as e:  # noqa: BLE001
      say(f"(no {name} sock: {e})")
  cp_gw = CANParser(ADAS_DBC, [("GW_Syn_All", 0)], 0)
  cp_eps = CANParser(ADAS_DBC, [("EPS_0x1C2", 0)], 0)

  trip = reset = 0
  have_sync = sync_ok = False
  last_full = {}      # source-label -> last decoded full counter (continuity)
  cont = {}           # source-label -> [plus1, jump, undec]
  last_oem = None     # (data) of most recent OEM bus-2 0x1D0, for SEED CHECK / alignment
  last_ourtx_ns = 0   # for engage-edge SEED CHECK on our direct 0x1D0
  last_eps_full = None  # last decoded counter the EPS actually received (bus0|0x80)
  last_eps_reset = None  # its reset window, so a legit GW-reset restart isn't flagged
  ring = deque(maxlen=RING)   # (ns, label, wire, rflag, full, mac_ok)
  last_sts = last_abort = None
  last_summary = time.monotonic()
  # --- Mode B (starvation) state ---
  last_eps_ns = 0          # monotonic ns we last saw an EPS-BOUND (bus0|0x80) 0x1D0
  last_ourtx_only_ns = 0   # ns we last saw our sendcan 0x1D0 request
  op = {"latActive": None, "enabled": None, "active": None,
        "controlsAllowed": None, "txBlocked": None, "rxInvalid": None}
  mode_b_open = False      # a starvation episode is currently being reported
  tx_blocked_base = None   # first txBlocked reading, to report deltas

  def read_op_state():
    """Pull latest op/panda state into `op`; log on change. Returns True if anything changed."""
    changed = False
    def setk(k, v):
      nonlocal changed
      if v is not None and op[k] != v:
        op[k] = v; changed = True
    cc = op_socks.get("carControl")
    if cc is not None:
      for m in messaging.drain_sock(cc, wait_for_one=False):
        try: setk("latActive", bool(m.carControl.latActive))
        except Exception: pass  # noqa: E722
        try: setk("enabled", bool(m.carControl.enabled))
        except Exception: pass  # noqa: E722
    for sname, field in (("selfdriveState", "active"), ("controlsState", "active")):
      s = op_socks.get(sname)
      if s is not None:
        for m in messaging.drain_sock(s, wait_for_one=False):
          try: setk("active", bool(getattr(getattr(m, sname), field)))
          except Exception: pass  # noqa: E722
    ps = op_socks.get("pandaStates")
    if ps is not None:
      for m in messaging.drain_sock(ps, wait_for_one=False):
        try:
          st = m.pandaStates[0]
        except Exception:  # noqa: BLE001
          continue
        try: setk("controlsAllowed", bool(st.controlsAllowed))
        except Exception: pass  # noqa: E722
        for cand in ("safetyTxBlocked", "txBufferOverflow", "safetyTxBlockedCnt"):
          try:
            setk("txBlocked", int(getattr(st, cand))); break
          except Exception: pass  # noqa: E722
        for cand in ("safetyRxChecksInvalid", "safetyRxInvalid"):
          try:
            setk("rxInvalid", bool(getattr(st, cand))); break
          except Exception: pass  # noqa: E722
    return changed

  def dump_ring():
    say(f"---- RING ({len(ring)} frames, decoded) ----", loud=True)
    now = time.monotonic_ns()
    for (fns, lab, w, rf, fl, ok) in ring:
      age = (now - fns) / 1e6
      say(f"    -{age:7.1f}ms {lab:5s} wire={w:2d} rflag={rf} full={fl if fl>=0 else 'UNDEC':>5} mac={'ok' if ok else 'BAD'}")
    say("---- end ring ----", loud=True)

  def note(label, dec, wire=None):
    c = cont.setdefault(label, [0, 0, 0])
    if dec is None or not dec[2]:
      c[2] += 1  # undecodable
      return None
    full = dec[1]
    prev = last_full.get(label)
    if prev is not None:
      step = full - prev
      if step == 1:
        c[0] += 1
      elif step != 0:
        c[1] += 1
    last_full[label] = full
    return full

  try:
    while True:
      events = []  # (ns, dir, addr, data, bus)
      can_batch = []
      for evt in messaging.drain_sock(can_sock, wait_for_one=False):
        can_batch.append((evt.logMonoTime, [(c.address, c.dat, c.src) for c in evt.can]))
        for c in evt.can:
          if c.address in WATCH:
            events.append((evt.logMonoTime, "rx", c.address, bytes(c.dat), c.src))
      for evt in messaging.drain_sock(sendcan_sock, wait_for_one=False):
        for c in evt.sendcan:
          if c.address in WATCH:
            events.append((evt.logMonoTime, "tx", c.address, bytes(c.dat), c.src))

      if can_batch:
        cp_gw.update(can_batch)
        cp_eps.update(can_batch)
        gw = cp_gw.vl["GW_Syn_All"]
        trip = (int(gw["Syn_TripCntrVal_Byte0_All"]) << 8) | int(gw["Syn_TripCntrVal_Byte1_All"])
        reset = ((int(gw["Syn_RstCntrVal_Byte0_All"]) << 16) | (int(gw["Syn_RstCntrVal_Byte1_All"]) << 8)
                 | int(gw["Syn_RstCntrVal_Byte2_All"]))
        mac = bytes([int(gw["Syn_MACInfo_Byte0_All"]), int(gw["Syn_MACInfo_Byte1_All"]), int(gw["Syn_MACInfo_Byte2_All"])])
        now_ok = (sync_mac(key, trip, reset) == mac)
        if now_ok and not have_sync:
          say(f"GW sync VERIFIED: key OK, trip={trip} reset={reset}.", loud=True)
        sync_ok, have_sync = now_ok, (have_sync or now_ok)

      events.sort(key=lambda e: e[0])
      for (ns, d, addr, data, src) in events:
        realbus = src & 0x7F
        fwd = bool(src & 0x80)   # 0x80 = transmitted/forwarded onto the bus (not a pure RX)
        # raw hex of everything watched ('*' = forwarded/TX copy)
        raw(f"{d} bus{realbus}{'*' if fwd else ' '} {addr:#05x}={hx(data)}")
        if not (have_sync and sync_ok):
          continue
        # classify 0x1D0-family frames by source and decode
        label = None
        if addr == A_1D0 and realbus == OEM_BUS and not fwd:
          label = "oem"; last_oem = data            # OEM's own frame on bus 2 (ground truth)
        elif addr == A_1D0 and realbus == OUR_BUS and fwd:
          label = "eps"                             # forwarded/injected onto bus 0 = EPS-bound
        elif addr == A_1D0 and d == "tx" and realbus == OUR_BUS:
          label = "ourtx"                           # our sendcan request (pre-panda)
        elif addr == A_CAND and realbus == CAND_BUS:
          label = "cand"                            # our candidate pool (bus 1)
        if label is None:
          continue

        dec = decode_1d0(key, data, trip, reset)
        full = note(label, dec)
        wire = (data[4] >> 2) & 0x3F
        rflag = data[4] & 0x3
        mac_ok = bool(dec and dec[2])
        ring.append((ns, label, wire, rflag, full if full is not None else -1, mac_ok))

        # THE KEY CHECK: what the EPS actually receives. A duplicate or backward counter
        # WITHIN THE SAME reset window is the replay the EPS rejects with abort=8. A
        # counter restart across a GW reset (reset window changes) is legit, not flagged.
        if label == "eps":
          if mode_b_open:  # EPS-bound stream resumed — close the episode
            resumed_gap = (ns - last_eps_ns) / 1e6
            say(f"MODE B cleared: EPS-bound 0x1D0 resumed after {resumed_gap:.0f}ms (full={full}).", loud=True)
            mode_b_open = False
          last_eps_ns = ns
        if label == "eps" and dec is not None and dec[2]:
          er = dec[0]  # decoded reset window of this EPS-bound frame
          if last_eps_full is not None and er == last_eps_reset and full <= last_eps_full:
            kind = "DUPLICATE" if full == last_eps_full else "BACKWARD"
            say(f"!!! EPS-BOUND {kind} (same window reset={er}): full {last_eps_full} -> {full} "
                f"<-- the replay the EPS aborts on", loud=True)
          last_eps_full, last_eps_reset = full, er
        elif label == "eps" and not mac_ok:
          say(f"!!! EPS-BOUND BAD MAC: wire={wire} rflag={rflag} (EPS can't verify this frame)", loud=True)
        if label == "ourtx":
          last_ourtx_only_ns = ns

        # SEED CHECK: first direct-TX (or candidate) after an idle gap vs OEM's last
        if label in ("ourtx", "cand") and ns - last_ourtx_ns > 50_000_000 and last_oem is not None:
          od = decode_1d0(key, last_oem, trip, reset)
          if od and od[2] and full is not None:
            diff = full - (od[1] + 1)
            say(f"SEED CHECK @engage ({label}): OEM_last={od[1]} our_first={full} "
                f"(expect {od[1]+1}) -> {'continuous OK' if diff==0 else f'OFF by {diff:+d}'}", loud=True)
        if label in ("ourtx", "cand"):
          last_ourtx_ns = ns

      # --- Mode B: op intent + panda state, and starvation detection ---
      if read_op_state():
        say(f"OP: latActive={op['latActive']} enabled={op['enabled']} active={op['active']} "
            f"ctrlAllowed={op['controlsAllowed']} txBlocked={op['txBlocked']} rxInvalid={op['rxInvalid']}")
      if op["txBlocked"] is not None and tx_blocked_base is None:
        tx_blocked_base = op["txBlocked"]
      now_ns = time.monotonic_ns()
      # Starvation: our TX is live but the EPS-bound 0x1D0 stream has gone silent, so
      # panda stopped putting our frame on bus 0 — either safety blocked the tx
      # (controlsAllowed False / txBlocked climbing) or a forward/relay drop.
      if (not mode_b_open and last_ourtx_only_ns and last_eps_ns
          and (now_ns - last_ourtx_only_ns) < 30_000_000
          and (now_ns - last_eps_ns) > 60_000_000):
        eps_gap = (now_ns - last_eps_ns) / 1e6
        txb = None if (op["txBlocked"] is None or tx_blocked_base is None) else op["txBlocked"] - tx_blocked_base
        say(f"!!! MODE B STARVATION: our TX live but EPS-bound 0x1D0 silent {eps_gap:.0f}ms "
            f"| latActive={op['latActive']} ctrlAllowed={op['controlsAllowed']} "
            f"txBlockedD={txb} rxInvalid={op['rxInvalid']} | last_eps_full={last_eps_full}", loud=True)
        dump_ring()
        mode_b_open = True

      # EPS state + abort ring dump
      if can_batch:
        sts = int(cp_eps.vl["EPS_0x1C2"]["EPS_AdasLatCtrlSts"])
        abort = int(cp_eps.vl["EPS_0x1C2"]["EPS_AbortFb"])
        if (abort not in (0, 1) and last_abort in (0, 1, None)) or (sts == 3 and last_sts != 3):
          say(f"EPS REJECT: sts {last_sts}->{sts}  abort {last_abort}->{abort}", loud=True)
          say(f"  OP@reject: latActive={op['latActive']} ctrlAllowed={op['controlsAllowed']} "
              f"txBlocked={op['txBlocked']} rxInvalid={op['rxInvalid']} "
              f"eps_silent={(now_ns - last_eps_ns)/1e6:.0f}ms", loud=True)
          dump_ring()
        elif sts != last_sts or abort != last_abort:
          say(f"EPS: sts {last_sts}->{sts}  abort {last_abort}->{abort}")
        last_sts, last_abort = sts, abort

      now = time.monotonic()
      if now - last_summary > 2.0:
        last_summary = now
        parts = " | ".join(f"{lab}:+1={c[0]}/jmp={c[1]}/undec={c[2]} last={last_full.get(lab)}"
                           for lab, c in sorted(cont.items()))
        say(f"summary sync={'OK' if sync_ok else '?'} trip={trip} reset={reset} || {parts or 'no frames'}")

      time.sleep(0.002)

  except KeyboardInterrupt:
    say("=== stopped ===", loud=True)
  finally:
    raw_f.close(); dec_f.close()


if __name__ == "__main__":
  main()
