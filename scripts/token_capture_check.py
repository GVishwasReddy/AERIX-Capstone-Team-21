#!/usr/bin/env python3
"""Experiment 2 of the evidence plan: can a captured aircraft give up a token?

Searches a raw SD-card image, a block device, a file or a directory tree for
an order token in every encoding the stack has ever written it in (raw bytes,
hex, HEX, base64), and prints each hit with its byte offset / file. Read-only.

Protocol (pass line fixed BEFORE the run: the token is NOT recoverable):

    # on the Pi, with an order pulled (so a live token exists)
    sudo systemctl stop aerix-ble aerix-gcs
    sudo dd if=/dev/mmcblk0 of=/mnt/usb/card.img bs=4M status=progress
    # anywhere - the token hex comes from the Firebase console, not the Pi
    scripts/token_capture_check.py /mnt/usb/card.img --token <hex>

Run it twice: once on a card imaged BEFORE this fix (expect hits: the old
active_order.json and active_order.used_*.json held the token in the clear),
once after (expect none - the live file is in /run, which is RAM). Keep both
outputs; together they are the evidence row.

Exit status: 0 = not found, 1 = FOUND, 2 = usage error.
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from drone_stack.ble_handshake.order_store import token_forms  # noqa: E402

CHUNK = 8 << 20


def scan_stream(fh, token: bytes, label: str) -> list[tuple[str, str, int]]:
    """(label, encoding, offset) for every hit; chunked with overlap so a hit
    straddling a chunk boundary is still found."""
    forms = token_forms(token)
    overlap = max(len(f) for f in forms.values()) - 1
    hits: list[tuple[str, str, int]] = []
    base, tail = 0, b""
    while True:
        chunk = fh.read(CHUNK)
        if not chunk:
            break
        buf = tail + chunk
        start = base - len(tail)
        for name, form in forms.items():
            i = buf.find(form)
            while i != -1:
                if i + len(form) > len(tail):          # not already reported
                    hits.append((label, name, start + i))
                i = buf.find(form, i + 1)
        tail = buf[-overlap:] if overlap else b""
        base += len(chunk)
    return hits


def scan_path(path: Path, token: bytes) -> list[tuple[str, str, int]]:
    if path.is_dir():
        hits = []
        for dirpath, _dirs, files in os.walk(path):
            for name in files:
                p = Path(dirpath) / name
                try:
                    with open(p, "rb") as fh:
                        hits += scan_stream(fh, token, str(p))
                except OSError:
                    continue
        return hits
    with open(path, "rb") as fh:
        return scan_stream(fh, token, str(path))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("target", type=Path, help="image, device, file or directory")
    ap.add_argument("--token", required=True, help="the order's deliveryToken, hex")
    a = ap.parse_args(argv)
    try:
        token = bytes.fromhex(a.token)
    except ValueError:
        print("--token must be hex", file=sys.stderr)
        return 2
    if len(token) < 8:
        print("--token is too short to search for meaningfully", file=sys.stderr)
        return 2
    hits = scan_path(a.target, token)
    for label, enc, off in hits:
        print(f"FOUND  {enc:6s}  {label}  @ {off}")
    print(f"{len(hits)} hit(s) for a {len(token)}-byte token in {a.target}: "
          f"{'RECOVERABLE' if hits else 'not recoverable'}")
    return 1 if hits else 0


if __name__ == "__main__":
    sys.exit(main())
