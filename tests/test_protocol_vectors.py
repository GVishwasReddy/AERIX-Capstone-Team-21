"""Known-answer vectors for the phone <-> drone BLE protocol (v1).

The same bytes are in docs/app/APP_CHANGES_PROMPT.md for the Flutter app's
own unit tests. If either side changes a MAC format, its vectors break first
- on its own side, before a phone and a drone ever disagree in a field.
"""
from __future__ import annotations

import pytest

pytest.importorskip("bluez_peripheral")

from drone_stack.ble_handshake import drone_ble_peripheral as ble  # noqa: E402

TOKEN = bytes.fromhex("000102030405060708090a0b0c0d0e0f")
NONCE = bytes.fromhex("101112131415161718191a1b1c1d1e1f")
PHONE = "/org/bluez/hci0/dev_AA_BB_CC_DD_EE_01"
LAT, LNG = 12.9716, 77.5946

AUTH = bytes.fromhex("e76b9e0fe4021d62ea97745ef43c654dc14698aa")
ACK = bytes.fromhex("bc069744ca7d1c384185df67f837eed794830553")
GPS_PAYLOAD = bytes.fromhex("31322e39373136303030302c37372e3539343630303030"
                            "000309e9ed9ca48b73d1aa26b660663cdf")
DROP = bytes.fromhex("b43a55e61a3be4af0e9eca5836e2dd39b690b6e5")
SIGNED_NOT_READY = bytes.fromhex("4e4f545f5245414459004aef21b857297aa151a662fd1455915a")
SIGNED_RELEASED = bytes.fromhex("52454c45415345440022b088e68e6280748a83d2d61e49195e")


@pytest.fixture
def svc(monkeypatch):
    monkeypatch.setattr(ble.os, "urandom", lambda n: NONCE[:n])
    monkeypatch.setattr(ble, "geofence_radius_m", lambda: 12.0)
    monkeypatch.setattr(ble, "get_drone_live_gps", lambda: ((LAT, LNG), "ok"))
    monkeypatch.setattr(ble, "gcs_pilot_override", lambda: False)
    monkeypatch.setattr(ble, "gcs_handshake_open", lambda: True)
    monkeypatch.setattr(ble, "gcs_flying_other_order", lambda oid: None)
    monkeypatch.setattr(ble, "_post_async", lambda payload: None)
    s = ble.DroneService(ble.ActiveOrder("V1", TOKEN, LAT, LNG), None)
    s._start_rssi = lambda device: None
    s._request_drop_worker = lambda *a, **k: None
    return s


def test_the_drone_accepts_exactly_the_published_bytes(svc):
    assert svc.on_nonce_read(PHONE) == NONCE
    svc.on_auth_write(AUTH, PHONE)
    assert svc.is_authenticated
    assert svc.on_ack_read(PHONE) == ACK
    assert GPS_PAYLOAD.split(b"\x00")[0] == b"12.97160000,77.59460000"
    for _ in range(ble.TEMPORAL_READINGS_REQUIRED):
        svc.on_gps_write(GPS_PAYLOAD, PHONE)
    assert svc.on_result_read(PHONE) == ble.RESULT_GEOFENCE_OK
    svc.on_drop_write(DROP, PHONE)
    assert svc._drop_request_pending is True          # every gate passed


def test_signed_results_match_the_published_bytes(svc):
    svc.on_nonce_read(PHONE)
    svc.on_auth_write(AUTH, PHONE)
    svc.last_result = ble.RESULT_NOT_READY
    assert svc.on_result_signed_read(PHONE) == SIGNED_NOT_READY
    svc.last_result = ble.RESULT_RELEASED
    assert svc.on_result_signed_read(PHONE) == SIGNED_RELEASED
