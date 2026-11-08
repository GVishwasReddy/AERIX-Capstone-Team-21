"""The attacker table (docs/security/THREAT_MODEL.md), one test per row.

Experiment 1 of the evidence plan, run on the bench in software: every attack
must end with the right NAMED refusal, the servo never commanded, and the
order's single-use token still on disk. Each test drives the REAL peripheral
end to end - nonce, AUTH, signed GPS, temporal window, geofence, DROP, the
drop worker, the ledger and the release - with only the hardware edges
replaced: the servo (recorded), the navigator's answer and the GCS state.
Threads run inline so a release, if one happens, happens inside the test.

The row ids (T1..) are the THREAT_MODEL.md ids; keep the two in step.
"""
from __future__ import annotations

import hashlib
import hmac
import json
import threading
import types

import pytest

pytest.importorskip("bluez_peripheral")
pytest.importorskip("cryptography")

from drone_stack.ble_handshake import drone_ble_peripheral as ble  # noqa: E402
from drone_stack.ble_handshake import order_store  # noqa: E402

TOKEN = bytes.fromhex("5ac1e0d27f3b9a4c8e61d0b2f4a7c935")
WRONG = bytes.fromhex("00112233445566778899aabbccddeeff")
PHONE = "/org/bluez/hci0/dev_AA_BB_CC_DD_EE_01"
ATTACKER = "/org/bluez/hci0/dev_66_66_66_66_66_66"
LAT, LNG = 12.9716, 77.5946
AIRBORNE_LEGACY = {"ok": False, "message": "precision drop disabled",
                   "data": {"mode": "legacy"}}
ON_GROUND = {"ok": False, "message": "disarmed - bench release",
             "data": {"mode": "legacy", "on_ground": True}}


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class InlineThread:
    def __init__(self, target=None, args=(), kwargs=None, **_):
        self._t, self._a, self._k = target, args, kwargs or {}

    def start(self):
        self._t(*self._a, **self._k)

    def is_alive(self):
        return False


@pytest.fixture
def rig(tmp_path, monkeypatch):
    """A peripheral with a real sealed order file, every gate open, and the
    drone hovering airborne over the recipient. Each test closes ONE gate (or
    plays one attacker) and checks nothing moved."""
    st = types.SimpleNamespace(pilot=False, window=True, other=None,
                               armed=True, nav=AIRBORNE_LEGACY,
                               drone=(LAT, LNG), servo=[], clock=Clock())
    order_file = order_store.write_order(tmp_path / "active_order.json", {
        "orderId": "ORDER_42", "deliveryToken": TOKEN.hex(),
        "targetLat": LAT, "targetLng": LNG})

    monkeypatch.setattr(ble, "_now", st.clock)
    monkeypatch.setattr(ble, "threading", types.SimpleNamespace(
        Thread=InlineThread, Lock=threading.Lock, Event=threading.Event))
    monkeypatch.setattr(ble.time, "sleep", lambda s: None)
    monkeypatch.setattr(ble, "geofence_radius_m", lambda: 12.0)
    monkeypatch.setattr(ble, "get_drone_live_gps", lambda: (st.drone, "ok"))
    monkeypatch.setattr(ble, "gcs_pilot_override", lambda: st.pilot)
    monkeypatch.setattr(ble, "gcs_handshake_open", lambda: st.window)
    monkeypatch.setattr(ble, "gcs_flying_other_order", lambda oid: st.other)
    monkeypatch.setattr(ble, "gcs_armed", lambda: st.armed)
    monkeypatch.setattr(ble, "request_precision_drop", lambda oid: st.nav)
    monkeypatch.setattr(ble, "_post_async", lambda payload: None)
    monkeypatch.setattr(ble, "gcs_payload_cfg",
                        lambda: {"channel": 13, "release_us": 1410, "lock_us": 1100})

    def servo(ch, pwm):
        st.servo.append((ch, pwm))
        return True
    monkeypatch.setattr(ble, "gcs_set_servo", servo)

    st.svc = ble.DroneService(ble.ActiveOrder.load(order_file), order_file)
    st.svc._start_rssi = lambda device: None
    st.order_file = order_file
    return st


# ── the phone's side of the protocol (what the Flutter app sends) ───────────

def mac(nonce: bytes, extra: bytes = b"", n: int = 20, key: bytes = TOKEN) -> bytes:
    return hmac.new(key, nonce + extra, hashlib.sha256).digest()[:n]


def gps(nonce: bytes, lat=LAT, lng=LNG, key: bytes = TOKEN) -> bytes:
    s = f"{lat:.8f},{lng:.8f}".encode()
    return s + b"\x00" + mac(nonce, s, 16, key)


def drop(nonce: bytes, lat=LAT, lng=LNG, key: bytes = TOKEN) -> bytes:
    return mac(nonce, f"{lat:.8f},{lng:.8f}".encode() + b"DROP", key=key)


def handshake(svc, device=PHONE, lat=LAT, lng=LNG, key=TOKEN):
    nonce = svc.on_nonce_read(device)
    svc.on_auth_write(mac(nonce, key=key), device)
    for _ in range(ble.TEMPORAL_READINGS_REQUIRED):
        svc.on_gps_write(gps(nonce, lat, lng, key), device)
    return nonce


def nothing_released(st) -> None:
    assert st.servo == [], "the servo was commanded"
    assert st.order_file.exists(), "the single-use token was spent"
    assert st.svc._delivered is False
    kinds = [json.loads(p.read_text())["body"]["kind"]
             for p in ble.get_ledger().pending.glob("*.json")]
    assert "delivery" not in kinds


# ── baseline: the legitimate recipient gets the parcel ──────────────────────

def test_T0_the_real_recipient_is_delivered_to(rig):
    nonce = handshake(rig.svc)
    rig.svc.on_drop_write(drop(nonce), PHONE)
    assert rig.svc.on_result_read(PHONE) == ble.RESULT_RELEASED
    assert rig.servo[0] == (13, 1410)                 # released...
    assert not rig.order_file.exists()                # ...and the token spent


# ── protocol attacks ────────────────────────────────────────────────────────

def test_T1_a_captured_session_replayed_later_is_refused(rig):
    # The phone completes AUTH+GPS but the window is shut (NOT_READY); an
    # attacker sniffed every write. Later it replays them on its own link.
    rig.window = False
    nonce = handshake(rig.svc)
    sniffed = (mac(nonce), gps(nonce), drop(nonce))
    rig.svc.on_drop_write(sniffed[2], PHONE)
    assert rig.svc.last_result == ble.RESULT_NOT_READY
    rig.clock.t += ble.SESSION_IDLE_S + 1            # the phone walks away
    rig.window = True                                # ...and the drone is ready

    rig.svc.on_nonce_read(ATTACKER)                  # fresh nonce, attacker's
    rig.svc.on_auth_write(sniffed[0], ATTACKER)
    for _ in range(ble.TEMPORAL_READINGS_REQUIRED):
        rig.svc.on_gps_write(sniffed[1], ATTACKER)
    rig.svc.on_drop_write(sniffed[2], ATTACKER)
    assert rig.svc.is_authenticated is False
    assert rig.svc.last_result == ble.RESULT_AUTH_FAIL
    nothing_released(rig)


def test_T2_a_replayed_drop_after_delivery_releases_nothing_more(rig):
    nonce = handshake(rig.svc)
    rig.svc.on_drop_write(drop(nonce), PHONE)
    fired = list(rig.servo)
    for _ in range(3):
        rig.svc.on_drop_write(drop(nonce), PHONE)
    assert rig.servo == fired                         # one release, ever
    assert rig.svc.on_result_read(PHONE) == ble.RESULT_RELEASED


def test_T3_a_stale_nonce_cannot_be_answered(rig):
    nonce = rig.svc.on_nonce_read(PHONE)
    rig.clock.t += ble.NONCE_TTL_S + 1
    rig.svc.on_auth_write(mac(nonce), PHONE)
    assert rig.svc.is_authenticated is False
    rig.svc.on_drop_write(drop(nonce), PHONE)        # dropped: no live session
    assert rig.svc.on_result_read(PHONE) == ble.RESULT_AUTH_FAIL
    nothing_released(rig)


def test_T4_the_wrong_order_token_never_authenticates(rig):
    nonce = handshake(rig.svc, key=WRONG)
    rig.svc.on_drop_write(drop(nonce, key=WRONG), PHONE)
    assert rig.svc.is_authenticated is False
    assert rig.svc.on_result_read(PHONE) == ble.RESULT_AUTH_FAIL
    nothing_released(rig)


def test_T5_a_second_phone_cannot_ride_the_recipients_session(rig):
    rig.window = False
    nonce = handshake(rig.svc)
    rig.window = True
    rig.svc.on_drop_write(drop(nonce), ATTACKER)
    assert rig.svc.on_result_read(ATTACKER) == ble.RESULT_AUTH_FAIL
    nothing_released(rig)
    assert rig.svc.on_result_read(PHONE) == ble.RESULT_GEOFENCE_OK  # untouched


def test_T6_tampered_coordinates_fail_their_signature(rig):
    nonce = rig.svc.on_nonce_read(PHONE)
    rig.svc.on_auth_write(mac(nonce), PHONE)
    genuine = gps(nonce)
    sig = genuine[genuine.index(0):]                  # 0x00 + the real signature
    forged = f"{LAT:.8f},{LNG + 0.0001:.8f}".encode() + sig   # ~11 m east
    rig.svc.on_gps_write(forged, PHONE)
    assert rig.svc.on_result_read(PHONE) == ble.RESULT_SIGNATURE_FAIL
    rig.svc.on_drop_write(drop(nonce), PHONE)
    nothing_released(rig)


def test_T7_a_drop_signed_for_other_coordinates_is_refused(rig):
    nonce = handshake(rig.svc)
    rig.svc.on_drop_write(drop(nonce, LAT + 0.00005, LNG), PHONE)
    assert rig.svc.on_result_read(PHONE) == ble.RESULT_SIGNATURE_FAIL
    nothing_released(rig)


def test_T8_a_relayed_session_from_a_distant_phone_fails_the_geofence(rig):
    # A relay forwards a genuine phone's writes verbatim, so every MAC holds -
    # but the phone signs ITS OWN position, ~220 m away. Bounded by the
    # geofence (and the person lock), not by the cryptography.
    far = LAT + 0.002
    nonce = handshake(rig.svc, device=ATTACKER, lat=far)
    rig.svc.on_drop_write(drop(nonce, far), ATTACKER)
    assert rig.svc.on_result_read(ATTACKER) == ble.RESULT_GEOFENCE_FAIL
    nothing_released(rig)


def test_T9_a_teleporting_fix_restarts_the_consistency_window(rig):
    nonce = rig.svc.on_nonce_read(PHONE)
    rig.svc.on_auth_write(mac(nonce), PHONE)
    far = LAT + 0.002
    for _ in range(ble.TEMPORAL_READINGS_REQUIRED - 1):
        rig.svc.on_gps_write(gps(nonce, far), PHONE)
    rig.svc.on_gps_write(gps(nonce), PHONE)           # jumps next to the drone
    assert rig.svc.on_result_read(PHONE).startswith(ble.RESULT_VERIFYING_PREFIX)
    rig.svc.on_drop_write(drop(nonce), PHONE)
    assert rig.svc.last_result == ble.RESULT_GEOFENCE_FAIL
    nothing_released(rig)


# ── mission-state attacks: valid cryptography, wrong moment ─────────────────

def test_T10_a_drop_on_the_ground_never_releases_a_real_order(rig):
    rig.armed, rig.nav = False, ON_GROUND
    nonce = handshake(rig.svc)
    rig.svc.on_drop_write(drop(nonce), PHONE)
    assert rig.svc.on_result_read(PHONE) == ble.RESULT_NOT_READY
    nothing_released(rig)


def test_T11_a_drop_during_a_pilot_takeover_is_refused_by_name(rig):
    nonce = handshake(rig.svc)
    rig.pilot = True
    rig.svc.on_drop_write(drop(nonce), PHONE)
    assert rig.svc.on_result_read(PHONE) == ble.RESULT_PILOT_OVERRIDE
    nothing_released(rig)


def test_T12_a_drop_before_a_person_is_locked_waits(rig):
    rig.window = False
    nonce = handshake(rig.svc)
    rig.svc.on_drop_write(drop(nonce), PHONE)
    assert rig.svc.on_result_read(PHONE) == ble.RESULT_NOT_READY
    nothing_released(rig)


def test_T13_a_token_for_another_order_cannot_release_this_flight(rig):
    rig.other = "ORDER_99"                    # the GCS is flying a different order
    nonce = handshake(rig.svc)
    rig.svc.on_drop_write(drop(nonce), PHONE)
    assert rig.svc.on_result_read(PHONE) == ble.RESULT_NOT_READY
    nothing_released(rig)


def test_T14_no_durable_audit_record_means_no_release(rig, monkeypatch):
    def broken():
        raise OSError("SD card read-only")
    monkeypatch.setattr(ble, "get_ledger", broken)
    nonce = handshake(rig.svc)
    rig.svc.on_drop_write(drop(nonce), PHONE)
    assert rig.svc.on_result_read(PHONE) == ble.RESULT_NOT_READY
    assert rig.servo == [] and rig.order_file.exists()


def test_T15_a_bystander_learns_nothing_from_result(rig):
    nonce = handshake(rig.svc)
    rig.svc.on_drop_write(drop(nonce), PHONE)
    assert rig.svc.on_result_read(ATTACKER) == ble.RESULT_AUTH_FAIL
    assert rig.svc.on_ack_read(ATTACKER) == bytes(20)


def test_T16_the_drone_proves_itself_to_the_phone(rig):
    # Mutual auth: the ACK is only computable with the token, so a fake
    # drone cannot collect the phone's GPS and DROP by impersonation.
    nonce = rig.svc.on_nonce_read(PHONE)
    rig.svc.on_auth_write(mac(nonce), PHONE)
    assert rig.svc.on_ack_read(PHONE) == mac(nonce, b"ACK")
    assert rig.svc.on_ack_read(PHONE) != mac(nonce, b"ACK", key=WRONG)


# ── cross-message confusion and the phone-side outcome ──────────────────────

def test_T17_one_messages_mac_cannot_stand_in_for_another(rig):
    # The MACs share a key and differ only in what follows the nonce, so the
    # separation is the PARSER's job (THREAT_MODEL.md, "domain separation").
    # Every overheard MAC, offered as a GPS signature over the bytes that
    # would make it match, must be refused without moving anything.
    nonce = rig.svc.on_nonce_read(PHONE)
    auth = mac(nonce)
    rig.svc.on_auth_write(auth, PHONE)
    ack = rig.svc.on_ack_read(PHONE)
    d = drop(nonce)
    gps_str = f"{LAT:.8f},{LNG:.8f}".encode()
    confusions = [
        b"" + b"\x00" + auth[:16],                    # AUTH  -> GPS over ""
        b"ACK" + b"\x00" + ack[:16],                  # ACK   -> GPS over "ACK"
        gps_str + b"DROP" + b"\x00" + d[:16],         # DROP  -> GPS over "<fix>DROP"
    ]
    for payload in confusions:
        rig.svc.on_gps_write(payload, PHONE)
        assert rig.svc.gps_verified is False
        assert rig.svc.gps_readings == []
    rig.svc.on_drop_write(auth, PHONE)                # AUTH replayed as DROP
    rig.svc.on_drop_write(ack, PHONE)                 # ACK replayed as DROP
    nothing_released(rig)


def _signed(rig, device, nonce, key=TOKEN):
    raw = rig.svc.on_result_signed_read(device)
    code, _, tag = raw.partition(b"\x00")
    ok = hmac.compare_digest(tag, mac(nonce, b"RESULT" + code, 16, key))
    return code, ok


def test_T18_the_phone_can_tell_a_real_released_from_a_forged_one(rig):
    nonce = handshake(rig.svc)
    assert _signed(rig, PHONE, nonce) == (ble.RESULT_GEOFENCE_OK, True)
    rig.svc.on_drop_write(drop(nonce), PHONE)
    assert _signed(rig, PHONE, nonce) == (ble.RESULT_RELEASED, True)
    # A fake drone answering RELEASED cannot produce the tag...
    forged = ble.RESULT_RELEASED + b"\x00" + bytes(16)
    code, _, tag = forged.partition(b"\x00")
    assert not hmac.compare_digest(tag, mac(nonce, b"RESULT" + code, 16))
    # ...nor replay the genuine one into a new session (fresh nonce).
    new_nonce = rig.svc.on_nonce_read(PHONE)
    old_tag = mac(nonce, b"RESULT" + ble.RESULT_RELEASED, 16)
    assert old_tag != mac(new_nonce, b"RESULT" + ble.RESULT_RELEASED, 16)


def test_T18b_no_session_no_signature(rig):
    handshake(rig.svc)
    raw = rig.svc.on_result_signed_read(ATTACKER)
    assert raw == ble.RESULT_AUTH_FAIL + b"\x00" + bytes(16)


# ── known-open: the phone as a signing oracle (found by Verifpal, 2026-10-08) ──

@pytest.mark.xfail(strict=True, reason="T29 OPEN until the app refuses nonces that "
                   "are not 16 bytes; the drone cannot tell these MACs from real ones")
def test_T29_a_phone_that_signs_any_challenge_lets_a_relay_choose_the_position(rig):
    # The phone answers AUTH = HMAC(token, challenge) for WHATEVER challenge a
    # fake drone shows it. Feed it the real drone's nonce + a position of the
    # attacker's choosing and its AUTH is a valid GPS signature / DROP MAC.
    def phone_auth(challenge: bytes) -> bytes:    # today's app: no length check
        return mac(challenge)
    far_phone = LAT + 0.01                        # recipient ~1.1 km away
    nonce = rig.svc.on_nonce_read(ATTACKER)       # attacker's device by the drone
    rig.svc.on_auth_write(phone_auth(nonce), ATTACKER)
    s = f"{LAT:.8f},{LNG:.8f}".encode()           # chosen: right under the drone
    for _ in range(ble.TEMPORAL_READINGS_REQUIRED):
        rig.svc.on_gps_write(s + b"\x00" + phone_auth(nonce + s)[:16], ATTACKER)
    rig.svc.on_drop_write(phone_auth(nonce + s + b"DROP"), ATTACKER)
    assert far_phone != LAT
    nothing_released(rig)                         # fails today: it IS released
