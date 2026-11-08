"""The BLE handshake session is bound to ONE central and dies when it goes quiet.

Before 2026-10-08 the peripheral kept one global session: whoever read NONCE
last owned it, any connected central could write AUTH/GPS/DROP against it, and
the nonce only changed when someone READ it. So a session sniffed off the air
(BLE writes here are unencrypted) stayed replayable for as long as nobody read
a fresh nonce - e.g. a phone that failed with NOT_READY and walked away. A
replay of its AUTH + GPS + DROP from another phone, once the drone was hovering
with the window open, passed every HMAC, and the replayed GPS (the real
recipient's position, next to the drone) passed the geofence too.

Two layers, each tested on its own:
  * device binding  - only the central that read the nonce may use it;
  * expiry          - an unanswered nonce, or an authenticated session that
                      has gone silent, is destroyed, so even a spoofed MAC
                      cannot replay a captured session later.
And the app contract must not move: one phone, read once, never re-read while
retrying DROP, keeps working however long the delivery takes.
"""
from __future__ import annotations

import hashlib
import hmac

import pytest

pytest.importorskip("bluez_peripheral")

from drone_stack.ble_handshake import drone_ble_peripheral as ble  # noqa: E402

TOKEN = bytes(range(16))
PHONE = "/org/bluez/hci0/dev_AA_BB_CC_DD_EE_01"
ATTACKER = "/org/bluez/hci0/dev_66_66_66_66_66_66"
LAT, LNG = 12.9, 77.6


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def env(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(ble, "_now", clock)
    monkeypatch.setattr(ble, "geofence_radius_m", lambda: 12.0)
    monkeypatch.setattr(ble, "get_drone_live_gps", lambda: ((LAT, LNG), "ok"))
    monkeypatch.setattr(ble, "gcs_pilot_override", lambda: False)
    monkeypatch.setattr(ble, "gcs_handshake_open", lambda: True)
    monkeypatch.setattr(ble, "gcs_flying_other_order", lambda oid: None)
    monkeypatch.setattr(ble, "_post_async", lambda payload: None)
    monkeypatch.setattr(ble.RssiSampler, "start", lambda self: None)
    svc = ble.DroneService(ble.ActiveOrder("ORDER_42", TOKEN, LAT, LNG), None)
    dispatched = []
    svc._request_drop_worker = lambda *a, **k: dispatched.append(a)
    return svc, clock, dispatched


def mac(nonce: bytes, extra: bytes = b"", n: int = 20) -> bytes:
    return hmac.new(TOKEN, nonce + extra, hashlib.sha256).digest()[:n]


def gps_payload(nonce: bytes) -> bytes:
    s = f"{LAT:.8f},{LNG:.8f}".encode()
    return s + b"\x00" + mac(nonce, s, 16)


def drop_payload(nonce: bytes) -> bytes:
    return mac(nonce, f"{LAT:.8f},{LNG:.8f}".encode() + b"DROP")


def handshake(svc, device=PHONE):
    """Read the nonce, authenticate and pass the geofence, like the app."""
    nonce = svc.on_nonce_read(device)
    svc.on_auth_write(mac(nonce), device)
    for _ in range(ble.TEMPORAL_READINGS_REQUIRED):
        svc.on_gps_write(gps_payload(nonce), device)
    return nonce


# ── the app contract ────────────────────────────────────────────────────────

def test_the_legitimate_phone_still_gets_all_the_way_to_dispatch(env):
    svc, _, dispatched = env
    nonce = handshake(svc)
    assert svc.on_ack_read(PHONE) == mac(nonce, b"ACK")
    assert svc.on_result_read(PHONE) == ble.RESULT_GEOFENCE_OK
    svc.on_drop_write(drop_payload(nonce), PHONE)
    assert len(dispatched) == 1


def test_an_active_session_never_expires_however_long_the_delivery(env):
    # Ten minutes of the app polling RESULT and retrying DROP every 2 s on
    # ONE nonce (it must never re-read it mid-delivery) stays authenticated.
    svc, clock, _ = env
    nonce = handshake(svc)
    for _ in range(300):
        clock.t += 2.0
        assert svc.on_result_read(PHONE) != ble.RESULT_AUTH_FAIL
    svc.on_drop_write(drop_payload(nonce), PHONE)
    assert svc._drop_request_pending is True


def test_the_limits_keep_10x_margin_over_the_measured_app():
    # logs/ble, 4-5 Oct 2026, 364 operations: nonce -> AUTH at most 0.32 s,
    # longest silence inside a live session 2.9 s. Tightening below 10x that
    # risks expiring a real phone mid-delivery; re-measure before moving them.
    assert ble.NONCE_TTL_S >= 10 * 0.32
    assert ble.SESSION_IDLE_S >= 10 * 2.9


def test_bluez_without_a_device_option_behaves_as_before(env):
    svc, _, dispatched = env
    nonce = handshake(svc, device=None)
    svc.on_drop_write(drop_payload(nonce), None)
    assert len(dispatched) == 1


# ── device binding ──────────────────────────────────────────────────────────

def test_a_second_central_cannot_authenticate_on_the_phones_nonce(env):
    svc, _, _ = env
    nonce = svc.on_nonce_read(PHONE)
    svc.on_auth_write(mac(nonce), ATTACKER)       # sniffed / relayed AUTH
    assert svc.is_authenticated is False


def test_a_second_central_cannot_drop_on_a_live_session(env):
    svc, _, dispatched = env
    nonce = handshake(svc)
    before = svc.last_result
    svc.on_drop_write(drop_payload(nonce), ATTACKER)
    assert dispatched == []
    # ...and it cannot disturb the real phone's session either.
    assert svc.last_result == before
    assert svc.is_authenticated is True
    assert svc.on_result_read(PHONE) == ble.RESULT_GEOFENCE_OK


def test_a_second_central_cannot_feed_gps_into_the_session(env):
    svc, _, _ = env
    nonce = svc.on_nonce_read(PHONE)
    svc.on_auth_write(mac(nonce), PHONE)
    for _ in range(ble.TEMPORAL_READINGS_REQUIRED):
        svc.on_gps_write(gps_payload(nonce), ATTACKER)
    assert svc.gps_verified is False
    assert svc.gps_readings == []


def test_the_outcome_and_the_ack_are_withheld_from_other_centrals(env):
    svc, _, _ = env
    handshake(svc)
    assert svc.on_result_read(ATTACKER) == ble.RESULT_AUTH_FAIL
    assert svc.on_ack_read(ATTACKER) == bytes(20)


def test_the_boot_nonce_belongs_to_nobody(env):
    # Nobody read it, so nobody can have computed an AUTH for it legitimately.
    svc, _, _ = env
    svc.on_auth_write(mac(svc.current_nonce), PHONE)
    assert svc.is_authenticated is False


# ── expiry ──────────────────────────────────────────────────────────────────

def test_an_unanswered_nonce_expires(env):
    svc, clock, _ = env
    nonce = svc.on_nonce_read(PHONE)
    clock.t += ble.NONCE_TTL_S + 1
    svc.on_auth_write(mac(nonce), PHONE)
    assert svc.is_authenticated is False


def test_a_captured_session_cannot_be_replayed_once_it_went_quiet(env):
    # The attacker spoofs the phone's MAC, so binding alone cannot stop this
    # one: only the expiry does. The phone failed (no person locked yet) and
    # walked away without reading a new nonce.
    svc, clock, dispatched = env
    nonce = handshake(svc)
    clock.t += ble.SESSION_IDLE_S + 1
    svc.on_auth_write(mac(nonce), PHONE)          # replayed AUTH
    for _ in range(ble.TEMPORAL_READINGS_REQUIRED):
        svc.on_gps_write(gps_payload(nonce), PHONE)
    svc.on_drop_write(drop_payload(nonce), PHONE)
    assert dispatched == []
    assert svc.current_nonce != nonce, "the dead nonce must be rotated away"


def test_an_order_swap_leaves_no_session_behind(env):
    svc, _, _ = env
    nonce = handshake(svc)
    svc.swap_order(ble.ActiveOrder("ORDER_43", TOKEN, LAT, LNG), None)
    svc.on_auth_write(mac(svc.current_nonce), PHONE)
    assert svc.is_authenticated is False
    assert svc.current_nonce != nonce


# ── the RSSI sampler gets the real device path ──────────────────────────────

def test_rssi_sampler_is_given_the_phones_device_path(env):
    # bluez_peripheral passes an options OBJECT, not a dict. options.get()
    # raised, a bare except swallowed it, and the sampler always got "".
    svc, _, _ = env
    opts = ble.CharacteristicWriteOptions({})
    opts._device = PHONE
    assert ble._device_of(opts) == PHONE
    handshake(svc)
    assert svc._rssi is not None and svc._rssi.device_path == PHONE


# ── logs never hold a MAC an attacker could reuse ───────────────────────────

def test_logs_never_contain_an_expected_mac_or_the_ack(env, capsys):
    svc, _, _ = env
    nonce = svc.on_nonce_read(PHONE)
    svc.on_auth_write(b"\x11" * 20, PHONE)        # wrong AUTH
    svc.on_auth_write(mac(nonce), PHONE)          # right AUTH -> ACK
    for _ in range(ble.TEMPORAL_READINGS_REQUIRED):
        svc.on_gps_write(gps_payload(nonce), PHONE)
    svc.on_drop_write(b"\x22" * 20, PHONE)        # wrong DROP
    out = capsys.readouterr().out
    for secret in (mac(nonce), mac(nonce, b"ACK"), drop_payload(nonce)):
        assert secret.hex() not in out
        assert secret[:8].hex() not in out
