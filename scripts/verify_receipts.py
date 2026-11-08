#!/usr/bin/env python3
"""Audit the drone's delivery ledger. Read-only: it never writes or moves a file.

    scripts/verify_receipts.py                       # repo-default locations
    scripts/verify_receipts.py --root <receipts/> --pub <key.pub> --orders <dir>

Reports, separately, because they mean different things:
  * v2 ledger  - signature of every record against the TRUSTED public key
                 (--pub, never the key a record claims to carry), sequence
                 gaps, hash-chain breaks, and intents with no outcome yet;
  * v1 legacy  - pre-2026-10-08 unsigned receipts (and whether they still hold
                 a raw token - scripts/scrub_receipt_tokens.py removes it);
  * corrupt    - 0-byte / zero-filled / unparsable files, i.e. a write torn by
                 power loss (two were found on the aircraft, 19 and 21 Sep);
  * reconcile  - orders whose token was spent (active_order.used_*.json) with
                 no receipt of any kind, and receipts with no spent token.
Exit status 0 only when the v2 chain is whole and nothing is corrupt.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from drone_stack.ble_handshake import receipt_ledger as rl  # noqa: E402

BLE = ROOT / "drone_stack" / "ble_handshake"


def classify(path: Path):
    raw = path.read_bytes()
    if not raw or not raw.strip(b"\x00"):
        return "corrupt", f"{len(raw)} bytes, all zero" if raw else "0 bytes"
    try:
        rec = json.loads(raw)
    except ValueError as exc:
        return "corrupt", f"unparsable ({exc})"
    if isinstance(rec, dict) and isinstance(rec.get("body"), dict):
        return "v2", rec
    if isinstance(rec, dict) and "orderId" in rec:
        return "v1", rec
    return "corrupt", "unrecognised layout"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", type=Path, default=BLE / "receipts")
    ap.add_argument("--pub", type=Path, default=BLE / "keys" / "drone_receipt_ed25519.pub",
                    help="the drone's TRUSTED public key (hex), kept off the aircraft")
    ap.add_argument("--orders", type=Path, default=BLE,
                    help="directory holding active_order.used_*.json")
    a = ap.parse_args()

    v2, v1, corrupt = [], [], []
    for sub in ("pending", "synced"):
        for p in sorted((a.root / sub).glob("*.json")):
            kind, val = classify(p)
            {"v2": v2, "v1": v1}.get(kind, corrupt).append((p, val))

    ok = True
    print(f"== v2 signed ledger: {len(v2)} record(s)")
    if v2:
        if not a.pub.exists():
            print(f"   !! no trusted public key at {a.pub}; cannot verify signatures")
            ok = False
        else:
            recs = [r for _, r in v2]
            problems = rl.verify_chain(recs, a.pub.read_text().strip())
            for prob in problems:
                print(f"   !! {prob}")
            ok &= not problems
            kinds = Counter(r["body"]["kind"] for r in recs)
            print("   " + ", ".join(f"{k} {n}" for k, n in sorted(kinds.items())))
            closed = {r["body"].get("intentSeq") for r in recs}
            dangling = [r["body"] for r in recs
                        if r["body"]["kind"] == rl.KIND_INTENT and r["body"]["seq"] not in closed]
            for b in dangling:
                print(f"   .. intent #{b['seq']} ({b['orderId']}) has no outcome yet "
                      "(closed as release_unconfirmed at the peripheral's next start)")
            if not problems:
                print("   chain whole: every signature valid, no gaps, no breaks")

    print(f"== v1 legacy receipts (unsigned, pre-ledger): {len(v1)}")
    with_token = [p for p, r in v1 if r.get("token")]
    if with_token:
        print(f"   !! {len(with_token)} still hold a raw delivery token "
              "(scripts/scrub_receipt_tokens.py)")

    print(f"== corrupt: {len(corrupt)}")
    for p, why in corrupt:
        print(f"   !! {p.name}: {why}")
    ok &= not corrupt

    spent = {}
    for p in a.orders.glob("active_order.used_*.json"):
        kind, val = classify(p)
        spent[p.name] = val.get("orderId") if kind in ("v1",) else None
    spent_ids = {v for v in spent.values() if v}
    receipted = {r["orderId"] for _, r in v1} | {
        r["body"]["orderId"] for _, r in v2 if r["body"]["kind"] == rl.KIND_DELIVERY}
    # A torn receipt's name still carries its orderId (<orderId>_<ts>.json).
    torn_ids = {p.name.rsplit("_", 1)[0] for p, _ in corrupt}
    print(f"== reconcile: {len(spent_ids)} spent token(s), {len(receipted)} receipted order(s)")
    for oid in sorted(spent_ids - receipted):
        note = "receipt TORN" if oid in torn_ids else "NO receipt"
        print(f"   !! {oid}: token spent, {note}")
    for oid in sorted(receipted - spent_ids):
        print(f"   .. {oid}: receipted, no spent-token file (bench token, or marker lost)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
