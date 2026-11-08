"""A captured aircraft must not give up the order token (threat model T6/T7).

Before 2026-10-08 ``firebase_sync.py pull-order`` wrote the token as hex into
``ble_handshake/active_order.json`` on the SD card: image the card, read the
token, forge the phone's side of the handshake. Now the live file sits in RAM
and the token inside it is sealed to the board. These tests pin both layers
and the capture check that turns them into evidence (experiment 2).
"""
from __future__ import annotations

import importlib.util
import io
import json
import os
import stat
from pathlib import Path

import pytest

pytest.importorskip("cryptography")

from drone_stack.ble_handshake import order_store as st  # noqa: E402

TOKEN = bytes.fromhex("8f3a1c5e9b2d4f60718293a4b5c6d7e8")
ORDER = {"orderId": "ORDER_42", "deliveryToken": TOKEN.hex(),
         "targetLat": 12.9, "targetLng": 77.6, "recipientId": "r1"}

_spec = importlib.util.spec_from_file_location(
    "token_capture_check",
    Path(__file__).resolve().parents[1] / "scripts" / "token_capture_check.py")
tcc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(tcc)


@pytest.fixture(autouse=True)
def board(monkeypatch):
    monkeypatch.setenv("AERIX_SEAL_SECRET", "board-A-serial")


# ── layer 2: sealed ─────────────────────────────────────────────────────────

def test_the_file_never_holds_the_token_in_any_encoding(tmp_path):
    path = st.write_order(tmp_path / "active_order.json", ORDER)
    blob = path.read_bytes()
    assert "deliveryToken" not in json.loads(blob)
    for name, form in st.token_forms(TOKEN).items():
        assert form not in blob, name


def test_it_round_trips_on_the_same_board(tmp_path):
    path = st.write_order(tmp_path / "o.json", ORDER)
    back = st.read_order(path)
    assert back["deliveryToken"] == TOKEN.hex()
    assert back["targetLat"] == 12.9 and back["orderId"] == "ORDER_42"


def test_a_file_copied_to_another_board_does_not_open(tmp_path, monkeypatch):
    path = st.write_order(tmp_path / "o.json", ORDER)
    monkeypatch.setenv("AERIX_SEAL_SECRET", "board-B-serial")
    with pytest.raises(ValueError):
        st.read_order(path)


def test_a_sealed_token_cannot_be_moved_to_another_order(tmp_path):
    path = st.write_order(tmp_path / "o.json", ORDER)
    data = json.loads(path.read_text())
    data["orderId"] = "ORDER_43"            # graft onto a different order
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        st.read_order(path)


def test_two_writes_of_one_token_share_no_ciphertext(tmp_path):
    a = json.loads(st.write_order(tmp_path / "a.json", ORDER).read_text())
    b = json.loads(st.write_order(tmp_path / "b.json", ORDER).read_text())
    assert a["deliveryTokenSealed"] != b["deliveryTokenSealed"]


def test_the_file_is_owner_only(tmp_path):
    path = st.write_order(tmp_path / "o.json", ORDER)
    assert stat.S_IMODE(os.stat(path).st_mode) == 0o600


def test_a_legacy_plain_file_still_loads(tmp_path):
    path = tmp_path / "o.json"
    path.write_text(json.dumps(ORDER))
    assert st.read_order(path)["deliveryToken"] == TOKEN.hex()


# ── layer 1: RAM ────────────────────────────────────────────────────────────

def test_aerix_order_dir_wins(tmp_path, monkeypatch):
    monkeypatch.setenv("AERIX_ORDER_DIR", str(tmp_path / "ram"))
    assert st.default_order_path() == tmp_path / "ram" / "active_order.json"


def test_without_root_it_prefers_the_runtime_dir_over_the_card(tmp_path, monkeypatch):
    monkeypatch.delenv("AERIX_ORDER_DIR", raising=False)
    monkeypatch.setattr(st, "SYSTEM_RUN_DIR", Path("/proc/forbidden/aerix"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path))
    assert st.order_dir() == tmp_path / "aerix-ble"


def test_spent_markers_leave_ram_for_persistent_storage(tmp_path, monkeypatch):
    monkeypatch.delenv("AERIX_SPENT_DIR", raising=False)
    monkeypatch.setattr(st, "is_volatile", lambda p: True)
    assert st.spent_dir(tmp_path / "active_order.json") == st.LEGACY_DIR
    monkeypatch.setattr(st, "is_volatile", lambda p: False)
    assert st.spent_dir(tmp_path / "active_order.json") == tmp_path


def test_dev_shm_is_recognised_as_ram():
    if not Path("/dev/shm").is_dir() or st.fs_type(Path("/dev/shm")) == "":
        pytest.skip("no /proc/mounts or /dev/shm here")
    assert st.is_volatile(Path("/dev/shm"))


# ── the peripheral reads it ─────────────────────────────────────────────────

def test_the_peripheral_loads_a_sealed_order(tmp_path):
    pytest.importorskip("bluez_peripheral")
    from drone_stack.ble_handshake import drone_ble_peripheral as ble
    path = st.write_order(tmp_path / "active_order.json", ORDER)
    order = ble.ActiveOrder.load(path)
    assert order.token == TOKEN and order.order_id == "ORDER_42"


def test_invalidating_a_sealed_order_keeps_the_commitment(tmp_path):
    pytest.importorskip("bluez_peripheral")
    from drone_stack.ble_handshake import drone_ble_peripheral as ble
    from drone_stack.ble_handshake.receipt_ledger import token_commit
    path = st.write_order(tmp_path / "active_order.json", ORDER)
    ble._invalidate_order_file(path)
    assert not path.exists()
    (marker,) = st.spent_dir(path).glob("active_order.used_*.json")
    assert json.loads(marker.read_text())["tokenCommit"] == token_commit(TOKEN)
    assert ble._spent_order_ids(path) == {"ORDER_42"}


# ── experiment 2's instrument ───────────────────────────────────────────────

@pytest.mark.parametrize("enc", ["raw", "hex", "HEX", "base64"])
def test_the_capture_check_finds_every_encoding(enc):
    form = st.token_forms(TOKEN)[enc]
    blob = os.urandom(1000) + form + os.urandom(1000)
    hits = tcc.scan_stream(io.BytesIO(blob), TOKEN, "img")
    assert (enc, 1000) in [(e, off) for _, e, off in hits]


def test_the_capture_check_finds_a_token_straddling_a_chunk_edge(monkeypatch):
    monkeypatch.setattr(tcc, "CHUNK", 64)
    form = st.token_forms(TOKEN)["hex"]
    blob = b"\x00" * 50 + form + b"\x00" * 100          # crosses offset 64
    hits = tcc.scan_stream(io.BytesIO(blob), TOKEN, "img")
    assert [(e, o) for _, e, o in hits] == [("hex", 50)]


def test_old_layout_is_recoverable_new_layout_is_not(tmp_path, monkeypatch):
    # The A/B of experiment 2, on a directory standing in for the card.
    old = tmp_path / "card_before"
    old.mkdir()
    (old / "active_order.json").write_text(json.dumps(ORDER))   # pre-fix pull
    assert tcc.main([str(old), "--token", TOKEN.hex()]) == 1

    new = tmp_path / "card_after"
    new.mkdir()
    st.write_order(new / "active_order.json", ORDER)            # even if on card
    assert tcc.main([str(new), "--token", TOKEN.hex()]) == 0
