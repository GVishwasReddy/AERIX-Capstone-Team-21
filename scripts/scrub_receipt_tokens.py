#!/usr/bin/env python3
"""Replace raw delivery tokens in pre-ledger files with a one-way commitment.

    scripts/scrub_receipt_tokens.py            # dry run: lists what it would change
    scripts/scrub_receipt_tokens.py --apply    # rewrite, durably, in place

Before 2026-10-08 every v1 receipt (receipts/*/<orderId>_<ts>.json, key
"token") and every spent order file (active_order.used_*.json, key
"deliveryToken") kept the raw token, so an imaged SD card gave up every token
this aircraft ever held. The value is replaced by
tokenCommit = sha256("AERIX/receipt/token-commit/v1" || token): whoever holds
the token can still prove which order a record belongs to; the card alone
reveals nothing.

Caveats, stated rather than hidden:
  * a v1 receipt's "signature" was keyed with the token, so after scrubbing it
    can only be checked by someone who holds the token (the order backend);
  * rewriting a file does not scrub the flash blocks the old copy occupied.
    Only never writing the token to persistent storage closes that.
Corrupt files are left alone (verify_receipts.py reports them).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from drone_stack.ble_handshake.receipt_ledger import durable_write, token_commit  # noqa: E402

BLE = ROOT / "drone_stack" / "ble_handshake"


def targets(ble_dir: Path):
    for sub in ("pending", "synced"):
        yield from ((p, "token") for p in sorted((ble_dir / "receipts" / sub).glob("*.json")))
    yield from ((p, "deliveryToken") for p in sorted(ble_dir.glob("active_order.used_*.json")))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dir", type=Path, default=BLE)
    ap.add_argument("--apply", action="store_true")
    a = ap.parse_args()

    changed = skipped = 0
    for path, key in targets(a.dir):
        try:
            rec = json.loads(path.read_bytes())
        except (OSError, ValueError):
            skipped += 1
            continue
        tok = rec.get(key) if isinstance(rec, dict) else None
        if not tok:
            continue
        try:
            commit = token_commit(bytes.fromhex(tok))
        except ValueError:
            print(f"   ?? {path.name}: {key} is not hex - left alone")
            skipped += 1
            continue
        del rec[key]
        rec["tokenCommit"] = commit
        rec["tokenScrubbed"] = True
        changed += 1
        print(f"   {'scrubbed' if a.apply else 'would scrub'} {path.name} ({key})")
        if a.apply:
            durable_write(path, json.dumps(rec, indent=2).encode("utf-8"),
                          mode=path.stat().st_mode & 0o777)
    print(f"{changed} file(s) {'scrubbed' if a.apply else 'to scrub'}, "
          f"{skipped} skipped (corrupt/unreadable)."
          + ("" if a.apply else "  Dry run - pass --apply to rewrite."))
    return 0


if __name__ == "__main__":
    sys.exit(main())
