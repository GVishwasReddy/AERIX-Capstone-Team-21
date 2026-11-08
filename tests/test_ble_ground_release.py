"""The BLE peripheral must never release a REAL order's parcel on the ground.

27 Sep 2026: after three failed flights the still-connected phone retried DROP
once the aircraft had landed. The navigator answered "disarmed - bench
release" (mode legacy) and the peripheral fired the servo at the launch point,
wrote a DELIVERED receipt and burned the token - orders 83aw, h857, OZsf.
Only the bench token (ActiveOrder.is_dev) may release on the ground.
"""
from __future__ import annotations

import pytest

pytest.importorskip("bluez_peripheral")

from drone_stack.ble_handshake import drone_ble_peripheral as ble  # noqa: E402


def _svc(monkeypatch, resp, armed, is_dev=False):
    order = ble.ActiveOrder("ORDER_42", b"\x00" * 16, 12.9, 77.6, is_dev=is_dev)
    svc = object.__new__(ble.DroneService)      # no BlueZ: only what the worker reads
    svc.order, svc.order_file = order, None
    svc.last_result = None
    svc._drop_request_pending = True
    svc._descent_in_progress = False
    calls = []
    svc._legacy_release = lambda: calls.append("release")
    svc._watch_precision_drop = lambda *a: calls.append("watch")
    monkeypatch.setattr(ble, "request_precision_drop", lambda oid: resp)
    monkeypatch.setattr(ble, "gcs_armed", lambda: armed)
    svc._request_drop_worker(order, None, 12.9, 77.6)
    return svc, calls


LANDED = {"ok": False, "message": "disarmed - bench release",
          "data": {"mode": "legacy", "on_ground": True}}
DISABLED_AIRBORNE = {"ok": False, "message": "precision drop disabled",
                     "data": {"mode": "legacy"}}


def test_real_order_on_the_ground_is_refused_and_keeps_its_token(monkeypatch):
    svc, calls = _svc(monkeypatch, LANDED, armed=False)
    assert calls == [], "servo fired on the ground for a real order"
    assert svc.last_result == ble.RESULT_NOT_READY
    assert svc._drop_request_pending is False


def test_telemetry_alone_refuses_when_the_navigator_does_not_flag_it(monkeypatch):
    # An older navigator (no on_ground flag) with the aircraft disarmed.
    old = {"ok": False, "message": "disarmed - bench release", "data": {"mode": "legacy"}}
    svc, calls = _svc(monkeypatch, old, armed=False)
    assert calls == [] and svc.last_result == ble.RESULT_NOT_READY


def test_the_bench_token_still_releases_on_the_ground(monkeypatch):
    svc, calls = _svc(monkeypatch, LANDED, armed=False, is_dev=True)
    assert calls == ["release"]


def test_airborne_legacy_release_is_unchanged(monkeypatch):
    svc, calls = _svc(monkeypatch, DISABLED_AIRBORNE, armed=True)
    assert calls == ["release"]


def test_precision_drop_accepted_is_watched(monkeypatch):
    ok = {"ok": True, "message": "precision drop started", "data": {"stage": "approach"}}
    svc, calls = _svc(monkeypatch, ok, armed=True)
    assert calls == ["watch"] and svc.last_result == ble.RESULT_DESCENDING
