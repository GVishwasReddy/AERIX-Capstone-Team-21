"""Every release the peripheral authorises is in the signed ledger, in order.

The property under test is an ORDERING, so each test watches the actuator and
the ledger together: a signed, fsynced release_intent must exist at the
instant the servo (or the navigator, which fires it itself) is asked to move,
and every way out must close that intent with what is actually known.
Fail-closed: no durable intent, no release, token kept.
"""
from __future__ import annotations

import json
import threading
import types

import pytest

pytest.importorskip("bluez_peripheral")
pytest.importorskip("cryptography")

from drone_stack.ble_handshake import drone_ble_peripheral as ble  # noqa: E402
from drone_stack.ble_handshake import receipt_ledger as rl  # noqa: E402

TOKEN = bytes(range(16))
OK = {"ok": True, "message": "precision drop started", "data": {}}
LEGACY = {"ok": False, "message": "precision drop disabled", "data": {"mode": "legacy"}}
LANDED = {"ok": False, "message": "disarmed - bench release",
          "data": {"mode": "legacy", "on_ground": True}}


class SyncThread:
    """threading.Thread that runs inline, so the release finishes in-test."""
    def __init__(self, target=None, args=(), kwargs=None, **_):
        self._t, self._a, self._k = target, args, kwargs or {}

    def start(self):
        self._t(*self._a, **self._k)


@pytest.fixture
def rig(tmp_path, monkeypatch):
    order_file = tmp_path / "active_order.json"
    order_file.write_text(json.dumps({"orderId": "ORDER_42",
                                      "deliveryToken": TOKEN.hex()}))
    order = ble.ActiveOrder("ORDER_42", TOKEN, 12.9, 77.6)
    svc = object.__new__(ble.DroneService)
    svc.order, svc.order_file = order, order_file
    svc.received_gps_lat, svc.received_gps_lng = 12.9, 77.6
    svc.last_result = None
    svc._delivered = False
    svc._drop_request_pending = True
    svc._descent_in_progress = False

    log = []                                   # actuator + ledger, interleaved
    real_append = rl.Ledger.append
    def append(self, kind, *a, **k):
        log.append(kind)
        return real_append(self, kind, *a, **k)
    monkeypatch.setattr(rl.Ledger, "append", append)
    monkeypatch.setattr(ble, "threading", types.SimpleNamespace(
        Thread=SyncThread, Lock=threading.Lock, Event=threading.Event))
    monkeypatch.setattr(ble, "_post_async", lambda payload: None)
    monkeypatch.setattr(ble, "gcs_armed", lambda: True)
    monkeypatch.setattr(ble, "gcs_pilot_override", lambda: False)
    monkeypatch.setattr(ble, "gcs_payload_cfg",
                        lambda: {"channel": 12, "release_us": 1410, "lock_us": 1100})
    monkeypatch.setattr(ble.time, "sleep", lambda s: None)

    def servo(ch, pwm):
        log.append(f"servo:{pwm}:token_file={'kept' if order_file.exists() else 'spent'}")
        return True
    monkeypatch.setattr(ble, "gcs_set_servo", servo)

    def run(resp):
        def ask(oid):
            log.append("navigator")
            return resp
        monkeypatch.setattr(ble, "request_precision_drop", ask)
        svc._request_drop_worker(order, order_file, 12.9, 77.6,
                                 evidence={"phoneLat": 12.9, "phoneLng": 77.6})
    return types.SimpleNamespace(svc=svc, log=log, run=run, order_file=order_file,
                                 monkeypatch=monkeypatch)


def ledger_records():
    led = ble.get_ledger()
    return [json.loads(p.read_text()) for p in sorted(led.pending.glob("*.json"))]


# ── ordering around the actuator ────────────────────────────────────────────

def test_legacy_release_intent_precedes_servo_and_token_precedes_servo(rig):
    rig.run(LEGACY)
    assert rig.log == [rl.KIND_INTENT, "navigator", "servo:1410:token_file=spent",
                       rl.KIND_DELIVERY, "servo:1100:token_file=spent"]
    assert ble.get_ledger().open_intents() == {}
    out = ledger_records()[-1]["body"]
    assert out["evidence"]["fcAccepted"] is True and out["evidence"]["channel"] == 12


def test_precision_drop_intent_precedes_asking_the_navigator(rig):
    # The navigator fires the servo itself once it says ok: the intent has to
    # be on disk before that question is even asked.
    rig.monkeypatch.setattr(ble, "DROP_POLL_S", 0)
    rig.monkeypatch.setattr(ble, "gcs_mission_state", lambda: {
        "drop_order_id": "ORDER_42", "drop_stage": "release", "drop_agl_m": 1.02})
    rig.run(OK)
    assert rig.log[:2] == [rl.KIND_INTENT, "navigator"]
    assert rig.log[-1] == rl.KIND_DELIVERY
    body = ledger_records()[-1]["body"]
    assert body["evidence"]["dropStage"] == "release"
    assert body["evidence"]["aglM"] == 1.02
    assert not rig.order_file.exists()


def test_the_intent_carries_the_authorising_evidence(rig):
    rig.run(LEGACY)
    ev = ledger_records()[0]["body"]["evidence"]
    assert ev == {"phoneLat": 12.9, "phoneLng": 77.6}


# ── fail closed ─────────────────────────────────────────────────────────────

def test_no_durable_intent_means_no_release_and_the_token_is_kept(rig):
    def dead_card(self, *a, **k):
        raise OSError("read-only file system")
    rig.monkeypatch.setattr(rl.Ledger, "append", dead_card)
    rig.run(LEGACY)
    assert rig.log == [], "the navigator was asked / the servo moved without a record"
    assert rig.order_file.exists()
    assert rig.svc.last_result == ble.RESULT_NOT_READY
    assert rig.svc._delivered is False


# ── every exit closes the intent with what is actually known ────────────────

@pytest.mark.parametrize("resp,why", [
    (None, "GCS unreachable"),
    ({"ok": False, "message": "no TF02 reading", "data": {}}, "navigator refused"),
    (LANDED, "on the ground"),
])
def test_refusals_close_the_intent_as_aborted_and_keep_the_token(rig, resp, why):
    rig.monkeypatch.setattr(ble, "gcs_armed", lambda: resp is not LANDED)
    rig.run(resp)
    assert rig.log[-1] == rl.KIND_ABORTED
    assert ledger_records()[-1]["body"]["evidence"]["why"] == why
    assert ble.get_ledger().open_intents() == {}
    assert rig.order_file.exists()
    assert not any(e.startswith("servo") for e in rig.log)


def test_a_navigator_abort_mid_descent_is_aborted(rig):
    rig.monkeypatch.setattr(ble, "DROP_POLL_S", 0)
    rig.monkeypatch.setattr(ble, "gcs_mission_state", lambda: {
        "drop_order_id": "ORDER_42", "drop_stage": "aborted", "drop_note": "lost TF02"})
    rig.run(OK)
    assert rig.log[-1] == rl.KIND_ABORTED
    assert rig.order_file.exists()


def test_losing_track_of_the_drop_is_unconfirmed_not_aborted(rig):
    # We stopped watching; we did not see the parcel stay aboard.
    rig.monkeypatch.setattr(ble, "DROP_POLL_S", 0)
    rig.monkeypatch.setattr(ble, "DROP_WATCH_MAX_S", 0.01)
    rig.monkeypatch.setattr(ble, "gcs_mission_state", lambda: None)
    rig.run(OK)
    assert rig.log[-1] == rl.KIND_UNCONFIRMED


def test_a_release_the_fc_did_not_accept_is_unconfirmed(rig):
    rig.monkeypatch.setattr(ble, "gcs_set_servo", lambda ch, pwm: False)
    rig.run(LEGACY)
    assert rig.log[-1] == rl.KIND_UNCONFIRMED
    assert ledger_records()[-1]["body"]["evidence"]["fcAccepted"] is False


def test_the_whole_chain_verifies_after_mixed_outcomes(rig):
    rig.run(None)
    rig.run({"ok": False, "message": "no TF02 reading", "data": {}})
    rig.run(LEGACY)
    led = ble.get_ledger()
    assert rl.verify_chain(ledger_records(), led.key.public_hex) == []


# ── the spent marker ────────────────────────────────────────────────────────

def test_the_spent_marker_holds_no_token_and_still_blocks_the_order(rig):
    rig.run(LEGACY)
    from drone_stack.ble_handshake import order_store
    markers = list(order_store.spent_dir(rig.order_file).glob("active_order.used_*.json"))
    assert len(markers) == 1
    raw = markers[0].read_text()
    assert TOKEN.hex() not in raw
    assert json.loads(raw)["tokenCommit"] == rl.token_commit(TOKEN)
    assert ble._spent_order_ids(rig.order_file) == {"ORDER_42"}
