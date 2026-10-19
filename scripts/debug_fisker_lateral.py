#!/usr/bin/env python3
"""
debug_fisker_lateral.py — live debug logger for lateral-engagement failures.

Runs ALONGSIDE sunnypilot (does not touch pandad, carcontroller, or anything
in the control path — just subscribes to the cereal 'can' topic that pandad
publishes, plus a few state topics). You can leave sunnypilot engaged / driving
while this runs.

Usage (on the comma 3X):
    scp this file to /data/openpilot/scripts/debug_fisker_lateral.py
    cd /data/openpilot
    PYTHONPATH=. python3 scripts/debug_fisker_lateral.py

Logs every state transition on the signals that matter for engaging lateral,
with microsecond-ish wall-clock timestamps:

  * MFS_0x514 button inputs (right/left stalk, roll wheel)
  * ADAS_0x313 cruise state (ADAS_Sts_ACC_ICC)
  * ADAS_0x1C0 our lateral-request fields (Req / Sts / Typ)
  * ADAS_0x1D0 our commanded steering angle (SteerAnReq)
  * EPS_0x1C2 EPS's reply (EPS_AdasLatCtrlSts) and abort reason (EPS_AbortFb)
  * EPS_0x1C4 driver-intervention bit (EPS_DrvrIntvSteerWhlDetd) + torque
  * VCU_0x358 basic CC state (VCU_Sts_CC_ICC)

Plus the sunnypilot/openpilot internal state as reported on cereal:
  * carState.cruiseState.{enabled, available, nonAdaptive}, steeringPressed
  * carControl.{enabled, latActive}
  * controlsState.{state, enabled}
  * selfdriveStateSP.mads.{state, enabled, active, available}
  * Params: FiskerMadsArmed, RemoteControlEnabled

Writes to /data/fisker_lateral_debug.log (append) AND stdout. Rotate / move
/data/fisker_lateral_debug.log before a session if you want a clean file.

Press Ctrl-C to stop.
"""

from __future__ import annotations

import datetime
import time
from pathlib import Path

# Use the sunnypilot-layout imports. The top-level `cereal` package on
# this device (if any) is a stale site-packages copy with an older API
# that doesn't export sub_sock; the real cereal we want is the one
# shipped under openpilot/cereal/.
from openpilot.cereal import messaging
from opendbc.can.parser import CANParser


ADAS_DBC = "fisker_ocean_adas"
ADAS_BUS = 0  # ADASBUS lives on bus 0 in the fisker port (see fisker-ocean-bus-scope memory)

# (signal_name, message_name) — the message is what we subscribe to in the
# CANParser; we grab the signal out of cp.vl[message_name][signal_name].
SIGNALS = [
  # MFSS buttons — WHICH button the user actually pressed. See
  # fisker-ocean-mfss-buttons memory for the vehicle-verified mapping.
  ("MFS_RiBtnNorth",            "MFS_0x514"),  # ADAS on/off
  ("MFS_RiBtnSouth",            "MFS_0x514"),  # long-press MADS (per user report)
  ("MFS_RiBtnEast",             "MFS_0x514"),  # MADS toggle
  ("MFS_RiBtnWest",             "MFS_0x514"),
  ("MFS_LeRollPress",           "MFS_0x514"),  # SET (engage cruise at current speed)
  ("MFS_LeRollUp",              "MFS_0x514"),
  ("MFS_LeRollDwn",             "MFS_0x514"),
  # ADAS cruise state
  ("ADAS_Sts_ACC_ICC",          "ADAS_0x313"),  # 0=Off 1=Standby 3=Active 5/6=standstill 9/10/11=fault
  # Our own lateral TX (what sunnypilot is asking EPS to do)
  ("ADAS_LatCtrl_Req",          "ADAS_0x1C0"),
  ("ADAS_LatCtrl_Sts",          "ADAS_0x1C0"),
  ("ADAS_LatCtrl_Typ",          "ADAS_0x1C0"),  # 1=LKA (only one Ocean honours), 3=TJA (DTC)
  ("ADAS_LatCtrl_SteerAnReq",   "ADAS_0x1D0"),  # commanded angle (deg)
  # EPS reply — the ground truth on whether lateral engaged
  ("EPS_AdasLatCtrlSts",        "EPS_0x1C2"),   # 0=Off 1=Available 2=Active 3=Fault
  ("EPS_AbortFb",               "EPS_0x1C2"),   # 0=none, 8=CAN abort (see secoc/alive-counter memories)
  ("EPS_DrvrIntvSteerWhlDetd",  "EPS_0x1C4"),   # conservative driver-overpower bit
  ("EPS_DrvrSteerTq",           "EPS_0x1C4"),
  # VCU basic CC state (test car runs CC not ACC per fisker-ocean-port-status)
  ("VCU_Sts_CC_ICC",            "VCU_0x358"),
]

# CANParser wants a list of (message_name, polling_frequency_hz) tuples (or
# just message names). We derive it from SIGNALS so there's one source of
# truth — add a signal above and this list updates. 0 Hz = don't warn on
# missing messages, which is what we want (some msgs are faulted / absent).
MESSAGES = []
_seen = set()
for _signame, _msgname in SIGNALS:
  if _msgname in _seen:
    continue
  _seen.add(_msgname)
  MESSAGES.append((_msgname, 0))


def fmt_ts() -> str:
  return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def pretty_val(v):
  """Floats to 2dp (angles), ints raw, everything else as-is."""
  if isinstance(v, float):
    return f"{v:.2f}"
  return str(v)


def main():
  log_path = Path("/data/fisker_lateral_debug.log")
  log_f = open(log_path, "a", buffering=1)  # line-buffered — tail -f works live

  def say(s: str, *, loud: bool = False) -> None:
    prefix = "***" if loud else "   "
    line = f"[{fmt_ts()}] {prefix} {s}"
    print(line, flush=True)
    log_f.write(line + "\n")

  say(f"=== fisker lateral debug — writing to {log_path} ===", loud=True)
  say("Subscribing to CAN bus 0 (ADASBUS) via cereal, plus cereal state topics...")

  cp = CANParser(ADAS_DBC, MESSAGES, ADAS_BUS)
  # Our TX frames (0x1D0 + 0x1C0 at 100 Hz) go out over 'sendcan', NOT 'can'
  # — 'can' is the RX side. Build a dedicated parser so we can see what the
  # carcontroller is actually transmitting on the wire. Same DBC, same bus.
  cp_tx = CANParser(ADAS_DBC, [("ADAS_0x1D0", 0), ("ADAS_0x1C0", 0)], ADAS_BUS)
  # OEM's OWN 0x1D0 on bus 2. While we're engaged the panda blocks these from
  # reaching the EPS (bus 2 -> bus 0), but the OEM ADAS keeps transmitting them
  # on bus 2 and pandad still reports them. This is the GROUND TRUTH for the
  # SecOC counter/reset the EPS expects: if our injected 0x1D0 (cp_tx, bus 0)
  # ever diverges from OEM's (cp_oem, bus 2), that divergence is exactly what
  # the EPS rejects. Comparing the two frame-by-frame pinpoints the reset
  # boundary race.
  cp_oem = CANParser(ADAS_DBC, [("ADAS_0x1D0", 0)], 2)
  # GW SecOC sync (0x20) carries the trip + reset counters. Watch the reset
  # field so we can time the ~1 Hz reset boundary against the EPS fault.
  cp_gw = CANParser(ADAS_DBC, [("GW_Syn_All", 0)], 0)
  # conflate=False so we see every CAN event (buttons at 50 Hz can get
  # coalesced away otherwise).
  can_sock = messaging.sub_sock("can", conflate=False, timeout=50)
  sendcan_sock = messaging.sub_sock("sendcan", conflate=False, timeout=50)

  sm = messaging.SubMaster([
    "carState", "carControl", "controlsState",
    "selfdriveState", "selfdriveStateSP", "carStateSP", "carControlSP",
  ])

  # Params for sunnypilot-side flags that gate engagement
  try:
    from openpilot.common.params import Params
    params = Params()
  except Exception as e:  # noqa: BLE001
    say(f"warning: couldn't open Params ({e}); FiskerMadsArmed/RemoteControlEnabled won't be logged")
    params = None

  # TX signals we track for diagnostic — our own counters and request bits.
  TX_SIGNALS = [
    ("ADAS_1D0_AliveCounter",     "ADAS_0x1D0"),
    ("ADAS_1D0_SSecOC_Fresh_Byte0", "ADAS_0x1D0"),
    ("ADAS_LatCtrl_SteerAnReq",   "ADAS_0x1D0"),
    ("ADAS_1C0_AliveCounter",     "ADAS_0x1C0"),
    ("ADAS_LatCtrl_Req",          "ADAS_0x1C0"),
    ("ADAS_LatCtrl_Sts",          "ADAS_0x1C0"),
    ("ADAS_LatCtrl_Typ",          "ADAS_0x1C0"),
  ]
  last_tx = {name: None for name, _ in TX_SIGNALS}

  last_can = {name: None for name, _ in SIGNALS}
  last_mads_state = None
  last_mads_active = None
  last_mads_available = None
  last_cs_cruise_enabled = None
  last_cc_enabled = None
  last_cc_lat_active = None
  last_cs_steer_pressed = None
  last_ctrls_state = None
  last_params = {}
  last_snapshot = 0.0
  last_gw_reset = None       # GW_Syn_All reset counter, for timing the ~1 Hz boundary
  last_divergence = None     # (ctr_gap, alv_gap, oem_resetbits, our_resetbits) ours-vs-OEM
  SNAPSHOT_PERIOD = 10.0  # s — print a full state dump this often even without transitions

  # ---- 0x1D0 TX cadence watchdog -------------------------------------------
  # The EPS wants our 0x1D0 at a steady 100 Hz (10 ms). A late frame leaves a gap
  # that an AUTOSAR SecOC RX can reject as a comms fault (AbortFb=8). We time the
  # gap between consecutive sendcan publishes that CARRY 0x1D0, using the capnp
  # logMonoTime (= card.py's publish time, authoritative regardless of this tool's
  # own drain cadence). A skipped controls_update cycle — e.g. carControl not
  # all_alive for a tick — publishes NO sendcan, so it shows up here as a ~20 ms+
  # gap. Flag anything past 1.5x nominal; that's the Python-side "are we fast
  # enough" signal to correlate against the abort timestamp.
  TX_GAP_WARN_MS = 15.0
  last_tx_1d0_ns = None
  tx_gap_max_ms = 0.0

  mads_state_names = ["disabled", "paused", "enabled", "softDisabling", "overriding"]

  try:
    while True:
      # Drain CAN and reshape into the (log_mono_time, [(addr, dat, src), ...])
      # tuples that opendbc.can.parser.CANParser.update() expects.
      # (The previous update_strings-of-raw-capnp-bytes API was removed.)
      can_packets = []
      for evt in messaging.drain_sock(can_sock, wait_for_one=False):
        frames = [(c.address, c.dat, c.src) for c in evt.can]
        can_packets.append((evt.logMonoTime, frames))
      # Also drain our own TX side ('sendcan') so we can print the
      # ADAS_1D0 / ADAS_1C0 counter values we are actually transmitting.
      # This is what EPS sees on the wire for lateral — if our alive /
      # SSecOC counters are miscomputed, this is where you can tell.
      tx_packets = []
      for evt in messaging.drain_sock(sendcan_sock, wait_for_one=False):
        frames = [(c.address, c.dat, c.src) for c in evt.sendcan]
        tx_packets.append((evt.logMonoTime, frames))
      # 0x1D0 TX cadence: measure the gap between consecutive publishes that
      # actually carry 0x1D0 (per-publish logMonoTime, so accurate even when we
      # drain several at once). A gap past TX_GAP_WARN_MS means Python was late
      # feeding the EPS — the thing we're trying to rule in/out as the abort cause.
      for log_ns, frames in tx_packets:
        if not any(addr == 0x1D0 for addr, _dat, _src in frames):
          continue
        if last_tx_1d0_ns is not None:
          gap_ms = (log_ns - last_tx_1d0_ns) / 1e6
          if gap_ms > tx_gap_max_ms:
            tx_gap_max_ms = gap_ms
          if gap_ms > TX_GAP_WARN_MS:
            say(f"TXGAP 0x1D0 {gap_ms:5.1f} ms since last (>{TX_GAP_WARN_MS:.0f}) "
                f"— LATE, EPS may reject; session max {tx_gap_max_ms:.1f} ms", loud=True)
        last_tx_1d0_ns = log_ns
      if tx_packets:
        cp_tx.update(tx_packets)
        for name, msg in TX_SIGNALS:
          try:
            v = cp_tx.vl[msg][name]
          except (KeyError, AttributeError):
            continue
          if v is None:
            continue
          prev = last_tx[name]
          if prev is None:
            last_tx[name] = v
            continue
          # Transitions on Req/Sts/Typ are loud; counters we only print on
          # non-monotonic skips (otherwise they'd flood every tick).
          loud = name in ("ADAS_LatCtrl_Req", "ADAS_LatCtrl_Sts", "ADAS_LatCtrl_Typ")
          if "AliveCounter" in name or "SSecOC" in name:
            # Expected diff per 10 ms = 1 (modulo 15 for alive, modulo 256
            # on the SSecOC byte — the on-wire fresh byte is OEM_wire<<2
            # per the DBC bit layout, so a +1 counter is +4 on the byte).
            # Flag anything that isn't a steady +1 (+4 for SSecOC byte).
            expected_step = 4 if "SSecOC" in name else 1
            modulus = 256 if "SSecOC" in name else 15
            diff = (int(v) - int(prev)) % modulus
            if diff != expected_step:
              say(f"TX  {name:30s}  {int(prev):>4} -> {int(v):<4}  (STEP {diff} ≠ {expected_step})", loud=True)
            last_tx[name] = v
            continue
          if v != prev:
            say(f"TX  {name:30s}  {pretty_val(prev):>6} -> {pretty_val(v)}", loud=loud)
            last_tx[name] = v

      if can_packets:
        cp.update(can_packets)
        cp_oem.update(can_packets)   # OEM's 0x1D0 on bus 2 (ground truth)
        cp_gw.update(can_packets)    # GW_Syn_All reset counter (bus 0)

        # --- GW reset-boundary timing -------------------------------------
        # Log the SecOC reset counter whenever it increments. This is the ~1 Hz
        # boundary where the EPS zeroes the per-window msg counter. Correlate
        # its timestamp with the OEM-vs-ours divergence and the EPS fault below.
        try:
          gw_rst = (int(cp_gw.vl["GW_Syn_All"]["Syn_RstCntrVal_Byte0_All"]) << 16) \
                 | (int(cp_gw.vl["GW_Syn_All"]["Syn_RstCntrVal_Byte1_All"]) << 8) \
                 | int(cp_gw.vl["GW_Syn_All"]["Syn_RstCntrVal_Byte2_All"])
        except (KeyError, AttributeError):
          gw_rst = None
        if gw_rst is not None and gw_rst != last_gw_reset:
          if last_gw_reset is not None:
            say(f"GW  Syn_RstCntr  {last_gw_reset} -> {gw_rst}   (reset boundary)", loud=True)
          last_gw_reset = gw_rst

        # --- OEM(bus2) vs ours(sendcan): the definitive continuation check ---
        # While we inject, our 0x1D0 should BE exactly what OEM would have sent
        # (same SSecOC counter byte + same alive). Any divergence is precisely
        # what the EPS rejects. Only meaningful while we're actually TXing.
        try:
          oem_byte = int(cp_oem.vl["ADAS_0x1D0"]["ADAS_1D0_SSecOC_Fresh_Byte0"])
          oem_alv = int(cp_oem.vl["ADAS_0x1D0"]["ADAS_1D0_AliveCounter"])
          our_byte = int(cp_tx.vl["ADAS_0x1D0"]["ADAS_1D0_SSecOC_Fresh_Byte0"])
          our_alv = int(cp_tx.vl["ADAS_0x1D0"]["ADAS_1D0_AliveCounter"])
        except (KeyError, AttributeError):
          oem_byte = oem_alv = our_byte = our_alv = None
        if our_byte is not None and last_cc_lat_active:
          # Report the signed gap in the SSecOC counter (top 6 bits) and alive.
          oem_ctr, our_ctr = oem_byte >> 2, our_byte >> 2
          ctr_gap = ((our_ctr - oem_ctr + 32) % 64) - 32       # signed, [-32,31]
          alv_gap = ((our_alv - oem_alv + 7) % 15) - 7         # signed, [-7,7]
          rb_oem, rb_our = oem_byte & 0x3, our_byte & 0x3
          cur_div = (ctr_gap, alv_gap, rb_oem, rb_our)
          if cur_div != last_divergence:
            tag = "OK (identical)" if cur_div == (0, 0, rb_oem, rb_oem) else "DIVERGED"
            say(f"CMP ours vs OEM(bus2): ctrGap={ctr_gap:+d} alvGap={alv_gap:+d} "
                f"resetBits oem={rb_oem} ours={rb_our}  [{tag}]",
                loud=(tag != "OK (identical)"))
            last_divergence = cur_div

        for name, msg in SIGNALS:
          try:
            v = cp.vl[msg][name]
          except (KeyError, AttributeError):
            continue
          if v is None:
            continue
          prev = last_can[name]
          if prev is None:
            last_can[name] = v
            continue
          # For floats (angle, torque) only log transitions that cross a
          # meaningful delta — otherwise we drown in noise.
          if isinstance(v, float):
            if abs(v - prev) < 0.5:
              continue
          elif v == prev:
            continue
          loud = name in (
            "MFS_RiBtnNorth", "MFS_RiBtnSouth", "MFS_RiBtnEast", "MFS_LeRollPress",
            "ADAS_LatCtrl_Req", "ADAS_LatCtrl_Typ",
            "EPS_AdasLatCtrlSts", "EPS_AbortFb",
            "EPS_DrvrIntvSteerWhlDetd",
            "ADAS_Sts_ACC_ICC", "VCU_Sts_CC_ICC",
          )
          say(f"CAN {name:28s}  {pretty_val(prev):>8} -> {pretty_val(v)}", loud=loud)
          last_can[name] = v

      # Cereal state transitions
      sm.update(0)

      if sm.updated["carState"]:
        cs = sm["carState"]
        if cs.cruiseState.enabled != last_cs_cruise_enabled:
          last_cs_cruise_enabled = cs.cruiseState.enabled
          say(f"carState.cruiseState.enabled -> {cs.cruiseState.enabled}", loud=True)
        if cs.steeringPressed != last_cs_steer_pressed:
          last_cs_steer_pressed = cs.steeringPressed
          say(f"carState.steeringPressed -> {cs.steeringPressed}  (steeringTorque={cs.steeringTorque:.2f})")

      if sm.updated["carControl"]:
        cc = sm["carControl"]
        if cc.enabled != last_cc_enabled:
          last_cc_enabled = cc.enabled
          say(f"carControl.enabled -> {cc.enabled}", loud=True)
        if cc.latActive != last_cc_lat_active:
          last_cc_lat_active = cc.latActive
          say(f"carControl.latActive -> {cc.latActive}", loud=True)

      if sm.updated["controlsState"]:
        try:
          ctrls_state_str = str(sm["controlsState"].state)
        except Exception:  # noqa: BLE001
          ctrls_state_str = "<unavailable>"
        if ctrls_state_str != last_ctrls_state:
          last_ctrls_state = ctrls_state_str
          say(f"controlsState.state -> {ctrls_state_str}")

      if sm.updated["selfdriveStateSP"]:
        mads = sm["selfdriveStateSP"].mads
        try:
          st_name = mads_state_names[int(mads.state)] if 0 <= int(mads.state) < len(mads_state_names) else str(mads.state)
        except Exception:  # noqa: BLE001
          st_name = str(mads.state)
        if (mads.state, mads.active, mads.available) != (last_mads_state, last_mads_active, last_mads_available):
          say(f"MADS state={st_name:<14s} enabled={mads.enabled} active={mads.active} available={mads.available}", loud=True)
          last_mads_state = mads.state
          last_mads_active = mads.active
          last_mads_available = mads.available

      # Params (polled; they don't publish on cereal)
      if params is not None:
        for key in ("FiskerMadsArmed", "RemoteControlEnabled"):
          cur = params.get_bool(key)
          if last_params.get(key) != cur:
            last_params[key] = cur
            say(f"param {key} -> {cur}")

      # Periodic full snapshot — useful as a baseline when scrolling through
      # a long log; also catches signals that are set from startup and never
      # change again (so we'd otherwise see them nowhere).
      if time.monotonic() - last_snapshot > SNAPSHOT_PERIOD:
        last_snapshot = time.monotonic()
        snap_bits = []
        for name, _ in SIGNALS:
          v = last_can.get(name)
          if v is None:
            continue
          snap_bits.append(f"{name}={pretty_val(v)}")
        if snap_bits:
          say("snapshot CAN  " + "  ".join(snap_bits))
        if last_cs_cruise_enabled is not None:
          say(f"snapshot CS   cruise.enabled={last_cs_cruise_enabled}  cc.enabled={last_cc_enabled}  cc.latActive={last_cc_lat_active}")
        if last_mads_state is not None:
          try:
            st_name = mads_state_names[int(last_mads_state)] if 0 <= int(last_mads_state) < len(mads_state_names) else str(last_mads_state)
          except Exception:  # noqa: BLE001
            st_name = str(last_mads_state)
          say(f"snapshot MADS state={st_name}  active={last_mads_active}  available={last_mads_available}")
        say(f"snapshot TX   0x1D0 inter-frame max gap so far {tx_gap_max_ms:.1f} ms (nominal 10, warn >{TX_GAP_WARN_MS:.0f})")

      time.sleep(0.005)  # tight loop but don't peg a CPU core

  except KeyboardInterrupt:
    say("=== stopped by user ===", loud=True)
  finally:
    log_f.close()


if __name__ == "__main__":
  main()
