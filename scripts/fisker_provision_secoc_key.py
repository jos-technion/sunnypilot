#!/usr/bin/env python3
"""
fisker_provision_secoc_key.py — read the Ocean SecOC AES key from the radar ECU over
UDS and write it to params as 'SecOCKey'.

Run OFFROAD with openpilot stopped (this script needs exclusive panda access):
    tmux kill-session -t comma 2>/dev/null; pkill -f "system.manager" 2>/dev/null
    cd /data/openpilot && PYTHONPATH=. python3 scripts/fisker_provision_secoc_key.py

It:
  1. opens the panda in a diagnostic safety mode,
  2. (optionally) enters an extended session + does security access,
  3. ReadDataByIdentifier(KEY_DID) from the radar → 16-byte key,
  4. VERIFIES the key against a live GW_Syn_All (0x20) sync MAC before persisting,
  5. writes it to params only if it verifies.

!!! FILL IN THE CONFIG BLOCK BELOW with the radar's real UDS values before running. !!!
"""

from __future__ import annotations

import time

from panda import Panda
from opendbc.car.uds import UdsClient, SESSION_TYPE, ACCESS_TYPE, DATA_IDENTIFIER_TYPE
from opendbc.car.fisker.secoc import sync_mac, parse_sync

# ─────────────────────────── CONFIG (radar RE specifics) ───────────────────────────
# UDS request / response CAN ids for the RADAR ECU, and which bus it's on.
RADAR_TX_ADDR = 0x000      # <-- radar UDS request id        (TODO)
RADAR_RX_ADDR = 0x000      # <-- radar UDS response id        (TODO; often TX + 8)
RADAR_BUS     = 0          # <-- bus the radar answers on      (TODO: 0 or 2)

# DataIdentifier holding the 16-byte SecOC key (int, e.g. 0xF1A0).
KEY_DID = 0x0000           # <-- (TODO)

# Does reading the key require security access (service 0x27)? If so, set True and
# implement compute_key(). REQUEST_SEED/SEND_KEY cover level 1; for higher levels pass
# the odd (seed) / even (key) subfunction numbers.
NEEDS_SECURITY_ACCESS = False
SEED_ACCESS_TYPE = ACCESS_TYPE.REQUEST_SEED   # odd subfunction = request seed
KEY_ACCESS_TYPE  = ACCESS_TYPE.SEND_KEY       # even subfunction = send key

def compute_key(seed: bytes) -> bytes:
  """Seed→key for the radar's security access. TODO: implement the real algorithm."""
  raise NotImplementedError("fill in the radar seed->key algorithm")

# GW SecOC sync (0x20) lives on bus 0 (vehicle). Used only to VERIFY the read key.
SYNC_ADDR = 0x20
SYNC_BUS  = 0
# ────────────────────────────────────────────────────────────────────────────────────


def _capture_sync(p: Panda, timeout: float = 3.0):
  """Grab one GW_Syn_All (0x20) frame so we can verify the key before persisting."""
  p.can_clear(0xFFFF)
  t0 = time.monotonic()
  while time.monotonic() - t0 < timeout:
    for addr, dat, bus in p.can_recv():
      if addr == SYNC_ADDR and bus == SYNC_BUS and len(dat) >= 8:
        return bytes(dat[:8])
  return None


def main():
  if RADAR_TX_ADDR == 0x000 or KEY_DID == 0x0000:
    raise SystemExit("Fill in RADAR_TX_ADDR / RADAR_RX_ADDR / RADAR_BUS / KEY_DID first.")

  p = Panda()
  p.set_safety_mode(Panda.SAFETY_ELM327)   # diagnostic mode: allows UDS TX

  uds = UdsClient(p, RADAR_TX_ADDR, RADAR_RX_ADDR, bus=RADAR_BUS)

  print(f"radar UDS {hex(RADAR_TX_ADDR)}→{hex(RADAR_RX_ADDR)} bus {RADAR_BUS}; reading DID {hex(KEY_DID)}")
  try:
    uds.diagnostic_session_control(SESSION_TYPE.EXTENDED_DIAGNOSTIC)
  except Exception as e:  # noqa: BLE001 — some ECUs read the DID in default session
    print(f"  (extended session not entered: {e}; trying read anyway)")

  if NEEDS_SECURITY_ACCESS:
    seed = uds.security_access(SEED_ACCESS_TYPE)
    print(f"  seed: {seed.hex()}")
    uds.security_access(KEY_ACCESS_TYPE, security_key=compute_key(seed))
    print("  security access granted")

  key = uds.read_data_by_identifier(DATA_IDENTIFIER_TYPE(KEY_DID) if KEY_DID in iter(DATA_IDENTIFIER_TYPE) else KEY_DID)
  print(f"  radar returned {len(key)} bytes: {key.hex()}")
  if len(key) != 16:
    raise SystemExit(f"expected a 16-byte key, got {len(key)} — wrong DID or needs security access.")

  # ---- verify against a live GW sync MAC BEFORE writing ----
  sync = _capture_sync(p)
  if sync is None:
    print("WARNING: no GW_Syn_All (0x20) seen on the bus — cannot verify. NOT writing.")
    print("         (is the car on? is 0x20 on SYNC_BUS?)  Key read was:", key.hex())
    return
  trip, reset = parse_sync(sync)
  if sync_mac(key, trip, reset) != sync[5:8]:
    raise SystemExit(f"key FAILED verification against GW sync (trip={trip} reset={reset}). NOT writing.")
  print(f"  key VERIFIED against GW sync (trip={trip} reset={reset}).")

  # ---- persist ----
  from openpilot.common.params import Params
  Params().put("SecOCKey", key.hex())
  print("SecOCKey written to params. Restart openpilot to pick it up.")


if __name__ == "__main__":
  main()
