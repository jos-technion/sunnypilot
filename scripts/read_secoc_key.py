#!/usr/bin/env python3
"""Print the SecOC key stored in params. Run: PYTHONPATH=. python3 scripts/read_secoc_key.py"""
from openpilot.common.params import Params

raw = Params().get("SecOCKey")
if raw is None:
  print("SecOCKey: (not set)")
else:
  key = raw.decode().strip() if isinstance(raw, (bytes, bytearray)) else raw.strip()
  nbytes = len(bytes.fromhex(key)) if all(c in "0123456789abcdefABCDEF" for c in key) and len(key) % 2 == 0 else -1
  print(f"SecOCKey: {key}  ({nbytes} bytes)" if nbytes >= 0 else f"SecOCKey (raw): {key!r}")
