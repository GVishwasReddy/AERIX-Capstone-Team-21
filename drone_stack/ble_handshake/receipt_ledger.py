"""Write-ahead, Ed25519-signed, hash-chained delivery ledger.

Why this exists - two failures found on the aircraft's own SD card (2026-10-08):

  receipts/pending/1AKT1JW6..._1789804579.json   0 bytes      (19 Sep 13:26)
  receipts/pending/mlOBdbgG..._1789976404.json   299 x 0x00   (21 Sep 13:10)

Both are the ext4 signature of power lost right after an UNSYNCED write: the
size reached the journal, the data never reached the card. Both were written a
moment before the payload servo fired - the inrush the old code itself warned
"can brown out the Pi". In both cases the token-burning rename survived
(renames are journaled), so the order's token was spent AND its receipt lost:
exactly the combination the write-ordering comment claimed to prevent. Order
without fsync is not order.

And the old receipt carried the raw order token ("token": hex) to Firestore,
signed with an HMAC only the token holder could check.

What a record is now:
  * signed with the DRONE's own Ed25519 key - anyone with the public key can
    verify it; nobody without the private key can forge one;
  * chained - each record carries the SHA-256 of the previous one, so deleting,
    reordering or editing any record breaks verification from that point on;
  * durable - tmp file, fsync, atomic rename, fsync of the directory, BEFORE
    the call returns;
  * two-phase around the actuator - a ``release_intent`` is committed before
    the servo may move, and the outcome (``delivery`` / ``release_aborted``)
    after. A crash between the two leaves a dangling intent that
    ``recover()`` turns into a signed ``release_unconfirmed`` at the next
    boot. The audit trail can be pessimistic; it can no longer be silent.
  * token-free - it carries ``tokenCommit = SHA-256(domain || token)``. The
    order server, which issued the token, can bind the record to the order;
    the record itself reveals nothing that authenticates.

Pure Python + ``cryptography`` (already in the BLE venv). No BlueZ, so it is
tested directly (tests/test_receipt_ledger.py).
"""
from __future__ import annotations

import hashlib
import json
import os
import threading
import time
from pathlib import Path

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import (
    Ed25519PrivateKey, Ed25519PublicKey)
from cryptography.exceptions import InvalidSignature

LEDGER_VERSION = 2
GENESIS = "0" * 64
_TOKEN_DOMAIN = b"AERIX/receipt/token-commit/v1"

KIND_INTENT = "release_intent"
KIND_DELIVERY = "delivery"
KIND_ABORTED = "release_aborted"
KIND_UNCONFIRMED = "release_unconfirmed"
_OUTCOMES = (KIND_DELIVERY, KIND_ABORTED, KIND_UNCONFIRMED)


# ── durability ──────────────────────────────────────────────────────────────

def _fsync_dir(path: Path) -> None:
    try:
        fd = os.open(str(path), os.O_RDONLY)
    except OSError:
        return                       # platforms without directory fds
    try:
        os.fsync(fd)
    except OSError:
        pass
    finally:
        os.close(fd)


def durable_write(path: Path, data: bytes, mode: int = 0o644) -> None:
    """Atomically replace ``path`` with ``data`` and make it survive power loss.

    tmp in the same directory -> write -> fsync(file) -> rename -> fsync(dir).
    A reader (or a reboot) sees the old file or the new one, never a torn or
    zero-filled one."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    fd = os.open(str(tmp), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, mode)
    try:
        view = memoryview(data)
        while view:
            view = view[os.write(fd, view):]
        os.fsync(fd)
    finally:
        os.close(fd)
    os.replace(tmp, path)
    _fsync_dir(path.parent)


def durable_rename(src: Path, dst: Path) -> None:
    """os.replace + fsync of the directory, so the rename itself is durable."""
    os.replace(src, dst)
    _fsync_dir(Path(dst).parent)


# ── canonical form ──────────────────────────────────────────────────────────

def canonical(body: dict) -> bytes:
    """The exact bytes that are signed. Sorted keys, no whitespace, so any
    verifier in any language reproduces them from the JSON."""
    return json.dumps(body, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True).encode("ascii")


def token_commit(token: bytes) -> str:
    return hashlib.sha256(_TOKEN_DOMAIN + bytes(token)).hexdigest()


def record_hash(record: dict) -> str:
    """What the NEXT record's ``prev`` must equal: covers body and signature."""
    return hashlib.sha256(canonical(record["body"]) +
                          bytes.fromhex(record["sig"])).hexdigest()


# ── the drone's signing key ─────────────────────────────────────────────────

class DroneKey:
    """The aircraft's Ed25519 identity for receipts.

    Created on first use (private key mode 600, directory 700); the public key
    is written beside it as ``.pub`` (hex) for the order server and for
    scripts/verify_receipts.py."""

    def __init__(self, private: Ed25519PrivateKey):
        self._sk = private
        raw = private.public_key().public_bytes(
            serialization.Encoding.Raw, serialization.PublicFormat.Raw)
        self.public_hex = raw.hex()
        self.key_id = hashlib.sha256(raw).hexdigest()[:16]

    @classmethod
    def load_or_create(cls, path: Path) -> "DroneKey":
        path = Path(path)
        if path.exists():
            sk = serialization.load_pem_private_key(path.read_bytes(), password=None)
            if not isinstance(sk, Ed25519PrivateKey):
                raise ValueError(f"{path} is not an Ed25519 private key")
            key = cls(sk)
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            try:
                os.chmod(path.parent, 0o700)
            except OSError:
                pass
            sk = Ed25519PrivateKey.generate()
            pem = sk.private_bytes(serialization.Encoding.PEM,
                                   serialization.PrivateFormat.PKCS8,
                                   serialization.NoEncryption())
            durable_write(path, pem, mode=0o600)
            key = cls(sk)
            print(f"  🔏 [LEDGER] new drone receipt key {key.key_id} -> {path}")
        pub = path.with_suffix(".pub")
        if not pub.exists() or pub.read_text().strip() != key.public_hex:
            durable_write(pub, (key.public_hex + "\n").encode(), mode=0o644)
        return key

    def sign(self, data: bytes) -> str:
        return self._sk.sign(data).hex()


def verify_signature(public_hex: str, data: bytes, sig_hex: str) -> bool:
    try:
        Ed25519PublicKey.from_public_bytes(bytes.fromhex(public_hex)).verify(
            bytes.fromhex(sig_hex), data)
        return True
    except (InvalidSignature, ValueError):
        return False


# ── the ledger ──────────────────────────────────────────────────────────────

class Ledger:
    """Append-only signed chain of release records.

    Records are files ``<pending_dir>/<seq:06d>_<kind>_<orderId>.json`` so the
    existing uploader (firebase_sync.push_receipts) picks them up and moves
    them to ``synced/`` unchanged. The chain head lives one level up, in
    ``<root>/chain_head.json``, so moving records never breaks the chain."""

    def __init__(self, root: Path, key: DroneKey, now=time.time):
        self.root = Path(root)
        self.pending = self.root / "pending"
        self.head_path = self.root / "chain_head.json"
        self.key = key
        self._now = now
        self._lock = threading.Lock()

    # head ----------------------------------------------------------------
    def head(self) -> dict:
        if self.head_path.exists():
            return json.loads(self.head_path.read_text())
        return {"seq": 0, "hash": GENESIS, "open_intents": {}}

    # append --------------------------------------------------------------
    def append(self, kind: str, order_id: str, token: bytes | None = None,
               **evidence) -> dict:
        with self._lock:
            head = self.head()
            body = {
                "v": LEDGER_VERSION,
                "kind": kind,
                "seq": head["seq"] + 1,
                "prev": head["hash"],
                "orderId": order_id,
                "ts": round(self._now(), 3),
                "keyId": self.key.key_id,
                "evidence": evidence,
            }
            if token is not None:
                body["tokenCommit"] = token_commit(token)
            intents = dict(head.get("open_intents", {}))
            if kind in _OUTCOMES:
                body["intentSeq"] = intents.pop(order_id, None)
            elif kind == KIND_INTENT:
                intents[order_id] = body["seq"]
            record = {"body": body, "sig": self.key.sign(canonical(body)),
                      "pub": self.key.public_hex}
            name = f"{body['seq']:06d}_{kind}_{order_id}.json"
            # Record first, head second. A crash between them leaves a record
            # the head does not know about yet: verify_chain still links it
            # (its prev is the old head), and the next append would fork -
            # so _reconcile_head() adopts it at startup.
            durable_write(self.pending / name,
                          json.dumps(record, indent=1, sort_keys=True).encode())
            durable_write(self.head_path, json.dumps({
                "seq": body["seq"], "hash": record_hash(record),
                "open_intents": intents}).encode())
            return record

    # two-phase helpers ----------------------------------------------------
    def intent(self, order_id: str, token: bytes, **evidence) -> dict:
        return self.append(KIND_INTENT, order_id, token, **evidence)

    def outcome(self, kind: str, order_id: str, token: bytes | None = None,
                **evidence) -> dict:
        if kind not in _OUTCOMES:
            raise ValueError(kind)
        return self.append(kind, order_id, token, **evidence)

    def open_intents(self) -> dict:
        return dict(self.head().get("open_intents", {}))

    def recover(self) -> list[dict]:
        """At startup: adopt any record the head missed, then close every
        intent that has no outcome as ``release_unconfirmed``.

        Pessimistic on purpose - the drone does not know whether the servo
        fired before the power went, so it says exactly that, signed."""
        self._reconcile_head()
        closed = []
        for order_id, seq in sorted(self.open_intents().items(), key=lambda kv: kv[1]):
            closed.append(self.outcome(
                KIND_UNCONFIRMED, order_id,
                note=f"intent #{seq} has no outcome - power or process lost "
                     "between authorisation and confirmation"))
        return closed

    def _reconcile_head(self) -> None:
        with self._lock:
            records = sorted(self._all_records(), key=lambda r: r["body"]["seq"])
            if not records:
                return
            head = self.head()
            last = records[-1]
            if last["body"]["seq"] <= head["seq"]:
                return
            intents = dict(head.get("open_intents", {}))
            for r in records:
                b = r["body"]
                if b["seq"] <= head["seq"]:
                    continue
                if b["kind"] == KIND_INTENT:
                    intents[b["orderId"]] = b["seq"]
                elif b["kind"] in _OUTCOMES:
                    intents.pop(b["orderId"], None)
            durable_write(self.head_path, json.dumps({
                "seq": last["body"]["seq"], "hash": record_hash(last),
                "open_intents": intents}).encode())
            print(f"  🧾 [LEDGER] head adopted record #{last['body']['seq']} "
                  "written just before a crash")

    def _all_records(self) -> list[dict]:
        out = []
        for sub in ("pending", "synced"):
            for p in (self.root / sub).glob("*.json"):
                try:
                    r = json.loads(p.read_text())
                except (OSError, ValueError):
                    continue
                if isinstance(r, dict) and r.get("body", {}).get("v") == LEDGER_VERSION:
                    out.append(r)
        return out


# ── verification ────────────────────────────────────────────────────────────

def verify_chain(records: list[dict], public_hex: str) -> list[str]:
    """Every problem with a set of v2 records; empty means the chain is whole.

    Checks: each signature against the drone's public key (not the key the
    record claims to carry), sequence with no gaps or duplicates, each
    ``prev`` equal to the hash of the record before it, and the genesis."""
    problems = []
    ordered = sorted(records, key=lambda r: r["body"]["seq"])
    prev_hash, expect = GENESIS, 1
    for r in ordered:
        b = r["body"]
        if b["seq"] != expect:
            problems.append(f"seq {b['seq']}: expected {expect} - record(s) missing")
            expect = b["seq"]
        if not verify_signature(public_hex, canonical(b), r.get("sig", "")):
            problems.append(f"seq {b['seq']}: BAD SIGNATURE (edited or forged)")
        if b["prev"] != prev_hash:
            problems.append(f"seq {b['seq']}: chain broken (prev does not match)")
        prev_hash, expect = record_hash(r), expect + 1
    return problems
