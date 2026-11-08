"""Write-ahead, signed, chained delivery ledger (receipt_ledger.py).

The receipts it replaces were found torn on the aircraft: one 0 bytes, one 299
zero bytes - power lost after an unsynced write, just before the servo fired,
while the token-burning rename survived. These tests pin the properties that
make that impossible to repeat silently, and the ones an examiner will ask for:
a receipt nobody can forge, edit, delete or reorder without it showing.
"""
from __future__ import annotations

import json
import os
import stat

import pytest

pytest.importorskip("cryptography")

from drone_stack.ble_handshake import receipt_ledger as rl  # noqa: E402

TOKEN = bytes(range(16))


@pytest.fixture
def ledger(tmp_path):
    key = rl.DroneKey.load_or_create(tmp_path / "keys" / "drone_receipt_ed25519.pem")
    clock = iter(range(1_790_000_000, 1_800_000_000))
    return rl.Ledger(tmp_path / "receipts", key, now=lambda: next(clock))


def records(ledger):
    return [json.loads(p.read_text()) for p in sorted(ledger.pending.glob("*.json"))]


def deliver(ledger, order="ORDER_1"):
    ledger.intent(order, TOKEN, phoneLat=12.9, phoneLng=77.6)
    ledger.outcome(rl.KIND_DELIVERY, order, TOKEN, dropStage="release", aglM=1.02)


# ── durability: the bug that was found on the SD card ───────────────────────

def test_data_is_fsynced_before_the_rename_makes_it_visible(tmp_path, monkeypatch):
    events = []
    real_fsync, real_replace = os.fsync, os.replace
    monkeypatch.setattr(rl.os, "fsync", lambda fd: (events.append("fsync"), real_fsync(fd)))
    monkeypatch.setattr(rl.os, "replace",
                        lambda a, b: (events.append("replace"), real_replace(a, b)))
    rl.durable_write(tmp_path / "r.json", b"{}")
    assert events[:2] == ["fsync", "replace"], events     # data, THEN visible
    assert events[2:] == ["fsync"], "the directory entry must be synced too"


def test_no_temporary_files_are_left_behind(tmp_path):
    rl.durable_write(tmp_path / "r.json", b"one")
    rl.durable_write(tmp_path / "r.json", b"two")
    assert [p.name for p in tmp_path.iterdir()] == ["r.json"]
    assert (tmp_path / "r.json").read_bytes() == b"two"


# ── nothing that authenticates leaves the aircraft ──────────────────────────

def test_records_carry_a_commitment_never_the_token(ledger):
    deliver(ledger)
    for p in ledger.pending.glob("*.json"):
        raw = p.read_bytes()
        assert TOKEN.hex().encode() not in raw
        assert TOKEN not in raw
    assert records(ledger)[0]["body"]["tokenCommit"] == rl.token_commit(TOKEN)


def test_the_private_key_is_owner_only_and_stable(tmp_path):
    path = tmp_path / "keys" / "k.pem"
    a = rl.DroneKey.load_or_create(path)
    b = rl.DroneKey.load_or_create(path)
    assert a.public_hex == b.public_hex
    assert stat.S_IMODE(path.stat().st_mode) == 0o600
    assert path.with_suffix(".pub").read_text().strip() == a.public_hex


# ── tamper evidence ─────────────────────────────────────────────────────────

def test_an_untouched_chain_verifies(ledger):
    deliver(ledger, "A")
    deliver(ledger, "B")
    assert rl.verify_chain(records(ledger), ledger.key.public_hex) == []


def test_editing_a_record_is_detected(ledger):
    deliver(ledger)
    recs = records(ledger)
    recs[1]["body"]["evidence"]["aglM"] = 0.2           # "it was released lower"
    assert any("BAD SIGNATURE" in p for p in rl.verify_chain(recs, ledger.key.public_hex))


def test_deleting_a_record_is_detected(ledger):
    deliver(ledger, "A")
    deliver(ledger, "B")
    recs = records(ledger)
    del recs[1]                                       # hide A's delivery
    problems = rl.verify_chain(recs, ledger.key.public_hex)
    assert any("missing" in p for p in problems)
    assert any("chain broken" in p for p in problems)


def test_a_record_signed_by_another_key_is_rejected(ledger, tmp_path):
    deliver(ledger)
    forger = rl.DroneKey.load_or_create(tmp_path / "forger.pem")
    recs = records(ledger)
    recs[1]["sig"] = forger.sign(rl.canonical(recs[1]["body"]))
    recs[1]["pub"] = forger.public_hex               # the record's own claim
    assert rl.verify_chain(recs, ledger.key.public_hex) != []


# ── two-phase around the actuator ───────────────────────────────────────────

def test_an_outcome_closes_its_intent(ledger):
    ledger.intent("A", TOKEN)
    assert ledger.open_intents() == {"A": 1}
    ledger.outcome(rl.KIND_DELIVERY, "A", TOKEN)
    assert ledger.open_intents() == {}
    assert records(ledger)[1]["body"]["intentSeq"] == 1


def test_a_crash_between_intent_and_outcome_is_recorded_not_lost(ledger):
    ledger.intent("A", TOKEN)                         # ...power lost here
    closed = ledger.recover()                         # next boot
    assert [r["body"]["kind"] for r in closed] == [rl.KIND_UNCONFIRMED]
    assert ledger.open_intents() == {}
    assert ledger.recover() == []                     # idempotent
    assert rl.verify_chain(records(ledger), ledger.key.public_hex) == []


def test_a_crash_between_record_and_head_does_not_fork_the_chain(ledger, monkeypatch):
    real = rl.durable_write
    def die_on_head(path, data, mode=0o644):
        if path.name == "chain_head.json":
            raise OSError("power lost")
        real(path, data, mode)
    ledger.intent("A", TOKEN)
    monkeypatch.setattr(rl, "durable_write", die_on_head)
    with pytest.raises(OSError):
        ledger.outcome(rl.KIND_DELIVERY, "A", TOKEN)
    monkeypatch.setattr(rl, "durable_write", real)
    ledger.recover()                                  # adopts the orphan record
    deliver(ledger, "B")
    recs = records(ledger)
    assert rl.verify_chain(recs, ledger.key.public_hex) == []
    assert [r["body"]["kind"] for r in recs] == [
        rl.KIND_INTENT, rl.KIND_DELIVERY, rl.KIND_INTENT, rl.KIND_DELIVERY]
