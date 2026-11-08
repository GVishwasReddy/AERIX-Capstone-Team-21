#!/usr/bin/env python3
"""
Raspberry Pi 5 BLE Peripheral Service for Drone Handshake & Parcel Drop.

Security Features:
  1. HMAC-SHA256 Challenge-Response (Mutual Authentication) using a
     per-order dynamic token pulled from Firestore (see firebase_sync.py),
     not a hardcoded shared secret.
  2. Micro Geofence: phone GPS vs the DRONE's own live GPS (pulled from
     drone_stack's GCS at http://localhost:8090). There is no autonomous
     "follow-me" relocation: the drone never flies toward the phone, so only
     the recipient walking closer can pass this gate (removed 2026-09-27).
     If the drone's own position can't be read at all, the gate answers
     NO_DRONE_FIX rather than GEOFENCE_FAIL: nothing the recipient does
     would help, so saying "get closer" would be a lie.
  3. Anti-spoofing temporal consistency: a single GPS reading is never
     trusted on its own (easy to spoof/replay/glitch). The drone requires
     several consecutive readings in a row that agree with each other
     (no implausible jumps) within a short rolling window before a
     position is considered verified - see _record_temporal_reading().
  4. Signed GPS payload verification - prevents GPS coordinate tampering.
  5. A RESULT characteristic the phone must read after writing DROP, so a
     successful BLE write is never confused with a successful drop - the Pi
     is the only source of truth for whether the parcel actually released.
     Readable only by an authenticated peer, so the outcome of a delivery
     never leaks to an unauthenticated scanner in radio range.
  6. Single-use token: the order file is invalidated after a successful
     drop, and a signed local receipt is written for the dual-audit sync
     (see firebase_sync.py push-receipts).
  7. Precision drop (2026-09-26): once every gate passes, the navigator - not
     this process - flies the release: it moves ~1.8 m from the recipient,
     descends on the downward TF02-Pro to 1 m above the ground, fires the
     servo, reports the delivery complete, climbs to 5 m and RTLs, with the
     RPLIDAR hover avoidance live throughout. This process asks for it
     (``precision_drop``), answers DESCENDING, and only burns the token and
     writes the receipt once /api/state says the parcel has gone. The GCS
     owns the TF02-Pro's UART; nothing here opens it.
  8. RESULT_SIGNED (2026-10-08): the outcome with a MAC over this session's
     nonce, so the phone can tell a real RELEASED from a fake drone's. Order
     tokens live in RAM, sealed to the board (order_store.py). Attacks and the
     tests that pin each refusal: docs/security/THREAT_MODEL.md,
     tests/test_attack_table.py.

Dependencies:
    pip install -r requirements.txt   (bluez-peripheral)

Usage:
    sudo python3 drone_ble_peripheral.py                  # requires active_order.json
    sudo python3 drone_ble_peripheral.py --dev             # bench testing, no order file
    sudo python3 drone_ble_peripheral.py --order-file /path/to/active_order.json
"""

import os
import sys
import json
import time
import hmac
import hashlib
import asyncio
import argparse
import math
import threading
import http.client
import urllib.request
import urllib.error
from pathlib import Path

from bluez_peripheral.gatt.service import Service
from bluez_peripheral.gatt.characteristic import (
    characteristic, CharacteristicFlags, CharacteristicWriteOptions)
from bluez_peripheral.advert import Advertisement
from bluez_peripheral.agent import NoIoAgent
from bluez_peripheral.util import Adapter, get_message_bus

try:                                   # imported as drone_stack.ble_handshake.*
    from .receipt_ledger import (DroneKey, Ledger, durable_write, _fsync_dir,
                                 token_commit, KIND_DELIVERY, KIND_ABORTED,
                                 KIND_UNCONFIRMED)
    from . import order_store
except ImportError:                    # run as a script from this directory
    from receipt_ledger import (DroneKey, Ledger, durable_write, _fsync_dir,
                                token_commit, KIND_DELIVERY, KIND_ABORTED,
                                KIND_UNCONFIRMED)
    import order_store

# ══════════════════════════════════════════════════════════════════
#  CONSTANTS & UUIDs (Must match Flutter app)
# ══════════════════════════════════════════════════════════════════

SERVICE_UUID     = "12345678-1234-5678-1234-567812345678"
NONCE_CHAR_UUID  = "12345678-1234-5678-1234-567812345679"  # Read  — Challenge Nonce
AUTH_CHAR_UUID   = "12345678-1234-5678-1234-56781234567a"  # R/W   — Auth HMAC + Mutual ACK
DROP_CHAR_UUID   = "12345678-1234-5678-1234-56781234567b"  # Write — Drop Command
GPS_CHAR_UUID    = "12345678-1234-5678-1234-56781234567c"  # Write — GPS Coordinates
RESULT_CHAR_UUID = "12345678-1234-5678-1234-56781234567d"  # Read  — Outcome of last DROP attempt
# Read - the same outcome, MAC'd for the phone: code + 0x00 + HMAC(token,
# nonce + "RESULT" + code)[:16]. Additive (2026-10-08): an app that never reads
# it behaves exactly as before. See docs/security/THREAT_MODEL.md row T18.
RESULT_SIGNED_CHAR_UUID = "12345678-1234-5678-1234-56781234567e"

# ── GCS (drone_stack) integration ──
GCS_BASE_URL = os.environ.get("GCS_BASE_URL", "http://localhost:8090")
GCS_HTTP_TIMEOUT_S = 2.0

# ── GCS state cache (keeps blocking HTTP off the BLE event loop) ──
# Every characteristic handler runs ON the asyncio/D-Bus event loop, so a
# synchronous urlopen inside one stalls the entire BLE stack for up to
# GCS_HTTP_TIMEOUT_S - once per GPS write, and three times over per DROP write.
# With the phone streaming GPS at 1 Hz that was enough to break the write
# cadence, time out the phone's writes, and trip the BLE supervision timeout
# (i.e. manufacture the very disconnects the app has a reconnect path for).
# A single background thread now polls /api/state and the handlers read its
# snapshot.
GCS_STATE_POLL_S = 0.4
# A snapshot older than this is treated as no data at all: every gate that
# depends on it fails CLOSED rather than acting on a stale view of the aircraft.
GCS_STATE_MAX_AGE_S = 3.0

# ── Geofence radius ──
# Phone vs drone's own live GPS. Fallback only - the live value is
# delivery.ble_geofence_m, read from /api/state (geofence_radius_m()). It was
# 5.0 m, tighter than two independent GPS receivers can agree, and never passed.
GEOFENCE_MICRO_RADIUS_M = 12.0
# The phone streams GPS at ~1 Hz, so the geofence verdict is recomputed that
# often. Rate-limit the explanatory log to one line per this interval.
GEOFENCE_LOG_COOLDOWN_S = 8.0

# ── Autonomous relocation ("follow-me") — REMOVED 2026-09-27 ──
# The drone no longer flies toward the phone. It was disabled by the operator
# on 2026-09-26 (a commented-out goto_gps call) and has now been taken out
# entirely rather than left as dead code that the docstring still advertised.
# Consequence, stated plainly because it changes what the recipient must do:
# a micro-geofence gap does NOT close by itself. Only the recipient walking
# closer, or the navigator's own mission logic, can satisfy the gate.

# ── Anti-spoofing temporal consistency ──
# A single GPS reading is cheap to fake (fake-GPS apps, replay, multipath
# glitches). Require several consecutive readings that agree with each
# other before a position is trusted for a geofence decision.
TEMPORAL_READINGS_REQUIRED = 5   # consecutive consistent readings needed
TEMPORAL_WINDOW_S = 12.0         # readings older than this fall out of the window
# Bigger gap between consecutive readings = not the same person standing still.
# Kept at 10m deliberately: a phone held perfectly still still jitters ~3-8m
# between fixes (multipath near buildings, accuracy re-estimates), so a tighter
# bound would keep restarting the window and lock out legitimate customers.
TEMPORAL_MAX_JUMP_M = 10.0

# ── RESULT characteristic outcome codes (ASCII, read after writing DROP) ──
RESULT_PENDING        = b"PENDING"
# Retired 2026-09-27. It used to mean BOTH "the micro-geofence passed" (set by
# _check_geofences) and "the parcel has been released" (set by _legacy_release).
# The app resolved it by which branch of its loop happened to be reading, so a
# stale geofence-pass OK read during the DROP loop would have been taken as a
# completed delivery - receipt written, order marked DELIVERED, parcel still
# aboard. The two meanings are now RESULT_GEOFENCE_OK and RESULT_RELEASED.
# Kept only so an older peripheral's OK is still understood by a current app.
RESULT_OK             = b"OK"
# The micro-geofence passed. Says nothing about the parcel.
RESULT_GEOFENCE_OK    = b"GEOFENCE_OK"
# Position received and signature-verified, but the anti-spoofing window is
# still filling (n of TEMPORAL_READINGS_REQUIRED). Nothing has been judged yet,
# so this is NOT a failure: the recipient must hold still, not move. Sent as
# b"VERIFYING:<n>/<m>" so the app can show progress; match it by prefix.
RESULT_VERIFYING_PREFIX = b"VERIFYING"
RESULT_AUTH_FAIL      = b"AUTH_FAIL"
RESULT_GEOFENCE_FAIL  = b"GEOFENCE_FAIL"
RESULT_SIGNATURE_FAIL = b"SIGNATURE_FAIL"
# The phone's fix arrived and verified, but the DRONE cannot say where it is
# (GCS unreachable, or no 3D fix yet), so the micro-geofence cannot be judged
# at all. Distinct from GEOFENCE_FAIL, which means "you are too far away":
# these need opposite things from the recipient, and reporting both as
# GEOFENCE_FAIL made "my coordinates never arrived" look identical to
# "my coordinates arrived and were rejected". The token is NOT spent.
RESULT_NO_DRONE_FIX   = b"NO_DRONE_FIX"
# The drone has not locked onto a person at the drop point yet (or the window
# after the lock has closed). The token is NOT spent - retry the DROP.
RESULT_NOT_READY      = b"NOT_READY"
# Precision drop accepted: the drone is moving aside and descending to the
# release height. Keep reading RESULT - it becomes RELEASED (token spent) or
# NOT_READY (drop aborted, parcel still aboard, token kept - retry the DROP).
RESULT_DESCENDING     = b"DESCENDING"
RESULT_RELEASED       = b"RELEASED"      # servo fired at the release height
# The pilot has taken the aircraft (transmitter stick/switch movement, or a
# mode the navigator never commanded). The navigator has stood the delivery
# down and will not resume it on its own - only the operator can, from the GCS.
# Used to be reported as NOT_READY ("no person locked"), which made the app
# tell the recipient the drone was still looking for them for up to 60 s after
# the mission had already been aborted (27 Sep, 17:01 and 17:07). The token is
# NOT spent.
RESULT_PILOT_OVERRIDE = b"PILOT_OVERRIDE"

# ── Precision drop hand-off (navigator flies it; we watch /api/state) ──
DROP_POLL_S = 0.5
# Backstop only - the navigator bounds its own drop (drop_timeout_s + climb).
DROP_WATCH_MAX_S = 180.0

# ── Phone RSSI sampling (locating the recipient among several people) ──
RSSI_SAMPLE_S = 0.25             # controller read period
RSSI_POST_S = 0.5                # batch period to the GCS
RSSI_MAX_FAILS = 12              # ~3 s of no reading = the link is gone
RSSI_MAX_RUN_S = 20 * 60.0

# ── Session binding + expiry (2026-10-08) ──
# The nonce used to change only when somebody READ it, and any connected
# central could write against it. A session sniffed off the air therefore
# stayed replayable until someone happened to read a fresh nonce - and a
# replayed GPS fix (the real recipient's, next to the drone) passes the
# geofence. Now only the central that read the nonce may use it, and the
# nonce dies when it goes unanswered or the session goes quiet.
# Measured from logs/ble (4-5 Oct, 364 operations): nonce -> AUTH took at most
# 0.32 s and the longest gap inside a live session was 2.9 s (GPS stream), so
# both limits sit an order of magnitude above the slowest real phone.
NONCE_TTL_S = 10.0       # read NONCE -> write AUTH must happen within this
SESSION_IDLE_S = 30.0    # an authenticated session silent this long is destroyed

# Bound to a nonce nobody has read (boot, order swap, expiry). Compares unequal
# to every device path AND to None, so a BlueZ that omits "device" cannot
# match it either.
_NOBODY = object()

_HERE = Path(__file__).resolve().parent
_RECEIPTS_ROOT = _HERE / "receipts"

# ── Dev-only fallback (bench testing without Firestore/active_order.json) ──
_DEV_SECRET_KEY = b"MY_SUPER_SECRET_KEY"
_DEV_TARGET_LAT = 13.0827
_DEV_TARGET_LNG = 80.2707
_DEV_ORDER_ID = "DEV_BENCH_ORDER"


def _now() -> float:
    """Session clock (monotonic). A seam so tests can move time."""
    return time.monotonic()


def _device_of(options) -> str | None:
    """D-Bus path of the central behind this read/write, or None.

    bluez_peripheral hands handlers a Characteristic{Read,Write}Options
    OBJECT, not a dict. The old ``options.get("device")`` raised, a bare
    except swallowed it, and every caller silently got "" - which is why the
    RSSI sampler only ever ran on its single-link fallback."""
    dev = getattr(options, "device", None)
    if dev is None and isinstance(options, dict):
        dev = options.get("device")
    return str(dev) if dev else None


def _fp(data: bytes) -> str:
    """Short fingerprint of bytes RECEIVED over the air, for correlating log
    lines. Never call this on an expected/derived MAC: the log is on the SD
    card, and a stolen card must not hold a value that authenticates."""
    return hashlib.sha256(bytes(data)).hexdigest()[:8]


# ══════════════════════════════════════════════════════════════════
#  HAVERSINE DISTANCE CALCULATION
# ══════════════════════════════════════════════════════════════════

def haversine_distance(lat1, lng1, lat2, lng2):
    """
    Calculate the great-circle distance between two GPS points
    using the Haversine formula. Returns distance in meters.
    """
    R = 6371000  # Earth's radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lng2 - lng1)

    a = (math.sin(d_phi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c


# ══════════════════════════════════════════════════════════════════
#  ACTIVE ORDER LOADING (per-delivery dynamic token + target GPS)
# ══════════════════════════════════════════════════════════════════

class ActiveOrder:
    """The order this flight was dispatched for: dynamic token + target GPS.

    Written by ``firebase_sync.py pull-order`` before takeoff (the "GCS
    flashes token + target coordinates into the Pi's memory" step in the
    design doc). Loaded at startup, then replaced at runtime by _order_watch
    whenever the GCS takes on a new order (2026-09-24; it used to need a
    process restart per delivery, which nobody remembered to do).
    """

    def __init__(self, order_id: str, token: bytes, target_lat: float,
                 target_lng: float, is_dev: bool = False):
        self.order_id = order_id
        self.token = token
        self.target_lat = target_lat
        self.target_lng = target_lng
        # True only for the hardcoded bench token. Carried this far so the
        # handshake failure it guarantees can name itself - see
        # auth_characteristic.setter.
        self.is_dev = is_dev

    @classmethod
    def load(cls, path: Path) -> "ActiveOrder":
        # Unseals deliveryTokenSealed (order_store); a legacy plain file loads too.
        data = order_store.read_order(path)
        token_hex = data["deliveryToken"]
        return cls(
            order_id=data["orderId"],
            token=bytes.fromhex(token_hex),
            target_lat=float(data["targetLat"]),
            target_lng=float(data["targetLng"]),
        )

    @classmethod
    def dev_fallback(cls) -> "ActiveOrder":
        return cls(_DEV_ORDER_ID, _DEV_SECRET_KEY, _DEV_TARGET_LAT,
                   _DEV_TARGET_LNG, is_dev=True)


def _invalidate_order_file(path: Path) -> None:
    """Single-use token: retire the order file so it can't be reused.

    It used to be RENAMED to active_order.used_<ts>.json - token included - so
    every token this aircraft ever held sat on its SD card for good (a stolen
    or imaged card gave them all up). Now a token-free spent marker (orderId,
    spentAt, tokenCommit) is written durably FIRST, and only then is the order
    file removed. A crash between the two leaves both: the marker already
    makes _spent_order_ids() refuse that order, so the token stays unusable.

    Deleting a file does not scrub flash. The token never being written to
    persistent storage at all is the complete fix - order_store keeps the live
    file in RAM (/run/aerix-ble) and the marker on persistent storage
    (order_store.spent_dir), so it survives the reboot that clears the token."""
    if not path.exists():
        return
    used_path = (order_store.spent_dir(path)
                 / f"{path.stem}.used_{int(time.time())}{path.suffix}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        marker = {"orderId": data.get("orderId"), "spentAt": time.time()}
        try:
            token_hex = order_store.read_order(path).get("deliveryToken")
        except (ValueError, KeyError):
            token_hex = None         # unsealable: still retire the order
        if token_hex:
            marker["tokenCommit"] = token_commit(bytes.fromhex(token_hex))
        durable_write(used_path, json.dumps(marker, indent=2).encode("utf-8"))
        path.unlink()
        _fsync_dir(path.parent)
        print(f"[ORDER] Invalidated {path.name} -> {used_path.name} "
              "(token expired; spent marker holds no token)")
    except (OSError, ValueError) as exc:
        print(f"  ⚠️ Could not invalidate order file: {exc}")


# ══════════════════════════════════════════════════════════════════
#  GCS (drone_stack) HTTP HELPERS
# ══════════════════════════════════════════════════════════════════

#: There is deliberately no blocking `gcs_get` helper any more. Every read of
#: the aircraft's state goes through `gcs_state()` (the GcsStateCache snapshot),
#: because the only callers were characteristic handlers running on the BLE
#: event loop, where a synchronous urlopen stalls the radio link. If you need a
#: new field from /api/state, read it from the snapshot - do not add a GET here.


def gcs_post(path: str, payload: dict) -> dict | None:
    url = f"{GCS_BASE_URL}{path}"
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=GCS_HTTP_TIMEOUT_S) as response:
            return json.loads(response.read().decode())
    except (urllib.error.URLError, http.client.HTTPException, TimeoutError,
            OSError, ValueError) as exc:
        print(f"  ❌ [GCS] POST {path} failed: {exc}")
        return None


class GcsStateCache(threading.Thread):
    """Polls /api/state on its own thread so BLE handlers never block on HTTP.

    This is the whole point of the class: a characteristic handler must be a
    fast, non-blocking function because it runs on the same event loop that
    drives the BLE link. Reading the aircraft's state is therefore decoupled
    from answering the phone - handlers call `gcs_state()` and get whatever the
    last poll stored, or None if it is too old to trust.

    Fails closed by design: `gcs_state()` returns None when the newest snapshot
    is older than GCS_STATE_MAX_AGE_S, so a wedged or stopped GCS can never be
    mistaken for "everything is fine, carry on".
    """

    def __init__(self, period: float = GCS_STATE_POLL_S) -> None:
        super().__init__(daemon=True, name="gcs-state")
        self.period = period
        self.stop_evt = threading.Event()
        self._lock = threading.Lock()
        self._state: dict | None = None
        self._at = 0.0                 # monotonic time of the last good poll
        self._fail_streak = 0
        self._reported_down = False

    def run(self) -> None:
        while not self.stop_evt.is_set():
            state = self._poll()
            with self._lock:
                if isinstance(state, dict):
                    self._state, self._at = state, time.monotonic()
                    self._fail_streak = 0
                    if self._reported_down:
                        self._reported_down = False
                        print("  ✅ [GCS] /api/state reachable again")
                else:
                    self._fail_streak += 1
                    # One line when it goes down, not one per poll.
                    if self._fail_streak == 3 and not self._reported_down:
                        self._reported_down = True
                        print(f"  ⚠️ [GCS] /api/state unreachable at {GCS_BASE_URL} - "
                              "geofence and drop gates will fail closed until it returns")
            self.stop_evt.wait(self.period)

    def _poll(self) -> dict | None:
        """Quiet: this runs a few times a second, so it must not log per call."""
        try:
            with urllib.request.urlopen(f"{GCS_BASE_URL}/api/state",
                                        timeout=GCS_HTTP_TIMEOUT_S) as response:
                data = json.loads(response.read().decode())
        except (urllib.error.URLError, http.client.HTTPException, TimeoutError,
                OSError, ValueError):
            return None
        return data if isinstance(data, dict) else None

    def snapshot(self) -> tuple[dict | None, float]:
        """(state, age_in_seconds). state is None if nothing has been read yet."""
        with self._lock:
            if self._state is None:
                return None, math.inf
            return self._state, time.monotonic() - self._at

    def fresh(self) -> dict | None:
        state, age = self.snapshot()
        if state is None or age > GCS_STATE_MAX_AGE_S:
            return None
        return state


# Started in main(). Until then every gate that needs it fails closed, which is
# the correct posture for a process that has not yet seen the aircraft.
_GCS_CACHE: GcsStateCache | None = None


def gcs_state() -> dict | None:
    """Newest /api/state snapshot, or None if missing or too stale to trust.

    Never does I/O: safe to call from a characteristic handler.
    """
    cache = _GCS_CACHE
    return cache.fresh() if cache is not None else None


def gcs_state_age_note() -> str:
    """Why gcs_state() came back empty, for a log line the operator can act on."""
    cache = _GCS_CACHE
    if cache is None:
        return "state poller not started"
    state, age = cache.snapshot()
    if state is None:
        return f"no successful poll of {GCS_BASE_URL}/api/state yet"
    if age > GCS_STATE_MAX_AGE_S:
        return f"newest snapshot is {age:.1f}s old (max {GCS_STATE_MAX_AGE_S:.1f}s)"
    return "snapshot is current"


def gcs_handshake_open() -> bool:
    """May the parcel be released now? The navigator only opens the window
    once the camera has locked onto a person at the drop point (see
    NavigationNode._handshake_open). Fails CLOSED: no answer is a no."""
    state = gcs_state()
    if not isinstance(state, dict):
        return False
    mission = state.get("mission")
    return isinstance(mission, dict) and mission.get("handshake_open") is True


def gcs_pilot_override() -> bool:
    """Has the pilot taken control? The navigator latches this on any
    transmitter movement past its deadband and keeps it until the operator
    resumes from the GCS, so it is safe to read as "the delivery is halted".
    No state reads as False: this only ever REPLACES a refusal the other gates
    would give anyway with a more accurate one, it never opens anything."""
    mission = gcs_mission_state()
    return isinstance(mission, dict) and mission.get("pilot_override") is True


def gcs_armed() -> bool:
    """Is the aircraft armed, per the GCS telemetry? Fails CLOSED: no state,
    or no telemetry, reads as NOT armed - i.e. "on the ground"."""
    state = gcs_state()
    telemetry = (state or {}).get("telemetry")
    return isinstance(telemetry, dict) and telemetry.get("armed") is True


def gcs_flying_other_order(order_id: str) -> str | None:
    """The order id the GCS is flying, if it is a DIFFERENT one than ours.

    None when it matches, when the GCS holds no order, or when the aircraft is
    disarmed (the bench, where the --dev token is tested against whatever
    order the GCS happens to hold). A mismatch in flight means this token
    belongs to another flight - never release."""
    state = gcs_state()
    telemetry = (state or {}).get("telemetry")
    if not (isinstance(telemetry, dict) and telemetry.get("armed")):
        return None
    delivery = (state or {}).get("delivery")
    oid = delivery.get("order_id") if isinstance(delivery, dict) else None
    if oid and str(oid) != order_id:
        return str(oid)
    return None


def gcs_mission_state() -> dict | None:
    state = gcs_state()
    mission = (state or {}).get("mission")
    return mission if isinstance(mission, dict) else None


def request_precision_drop(order_id: str) -> dict | None:
    """Ask the navigator to fly the precision drop. None = GCS unreachable."""
    return gcs_post("/api/command", {"cmd": "precision_drop",
                                     "params": {"order_id": order_id}})


def _post_async(payload: dict) -> None:
    """Fire-and-forget POST, off the BLE/D-Bus event loop."""
    threading.Thread(target=gcs_post, args=("/api/command", payload), daemon=True).start()


class RssiSampler(threading.Thread):
    """Reads the live connection's RSSI and posts it to the GCS in batches.

    The navigator pairs each sample with where the aircraft was when it was
    measured (hence the wall-clock timestamp) and uses them to tell WHICH of
    the people under the drone is holding the phone."""

    def __init__(self, order_id: str, device_path: str) -> None:
        super().__init__(daemon=True)
        self.order_id = order_id
        self.device_path = device_path
        self.stop_evt = threading.Event()

    def run(self) -> None:
        try:
            from rssi_probe import RssiProbe, addr_from_device_path
        except Exception as exc:  # noqa: BLE001
            print(f"  ⚠️ [RSSI] probe unavailable: {exc}")
            return
        where = addr_from_device_path(self.device_path) or (0, None)
        probe = RssiProbe(*where)
        print(f"  📶 [RSSI] sampling hci{where[0]} {where[1] or '(single LE link)'}")
        batch, fails, n = [], 0, 0
        started = last_post = time.monotonic()
        try:
            while not self.stop_evt.is_set() and time.monotonic() - started < RSSI_MAX_RUN_S:
                v = probe.read()
                if v is None:
                    fails += 1
                    if fails >= RSSI_MAX_FAILS:
                        break
                else:
                    fails = 0
                    batch.append([round(time.time(), 3), v])
                    n += 1
                if batch and time.monotonic() - last_post >= RSSI_POST_S:
                    gcs_post("/api/command", {"cmd": "ble_phone_signal", "params": {
                        "order_id": self.order_id, "samples": batch}})
                    batch, last_post = [], time.monotonic()
                self.stop_evt.wait(RSSI_SAMPLE_S)
        finally:
            probe.close()
            print(f"  📶 [RSSI] sampler stopped after {n} samples")


def get_drone_live_gps() -> tuple[tuple[float, float] | None, str]:
    """The drone's own current GPS (Gate 3 needs this - the doc's "Pi asks the
    Pixhawk for the drone's OWN Live Satellite GPS").

    Returns ``(position, reason)``: position is None when it cannot be
    determined, and reason then says which of the several quite different
    causes it was. The caller reports that instead of a bare
    RESULT_GEOFENCE_FAIL - "the drone does not know where it is" and "you are
    30 m away" ask opposite things of the recipient, and collapsing them into
    one code is what made this undiagnosable from the app.

    Reads the cached snapshot only - no HTTP, so it is safe on the event loop.

    The real GCS has no ``/api/telemetry`` route - the one-shot snapshot is
    ``GET /api/state`` (see gcs/server.py), and its GPS fields live nested
    under a ``"position"`` key as ``lat``/``lon``/``fix`` (see
    GcsHub.build_payload), not a flat ``has_fix``/``lat``/``lon``.
    """
    state = gcs_state()
    if state is None:
        return None, f"GCS state unavailable: {gcs_state_age_note()}"
    position = state.get("position") or {}
    if not position.get("fix", False):
        return None, "drone has no 3D GPS fix yet (waiting on satellites)"
    lat, lon = position.get("lat"), position.get("lon")
    if lat is None or lon is None:
        return None, "GCS reported a fix but no lat/lon"
    try:
        return (float(lat), float(lon)), "ok"
    except (TypeError, ValueError):
        return None, f"GCS reported unparseable lat/lon: {lat!r}, {lon!r}"


def geofence_radius_m() -> float:
    """The micro-geofence radius from config (via /api/state), else the default.

    Read from the cached snapshot like gcs_payload_cfg(), so config/*.yaml is
    the one place the number lives.
    """
    state = gcs_state() or {}
    try:
        r = float((state.get("ble") or {}).get("geofence_m", GEOFENCE_MICRO_RADIUS_M))
    except (TypeError, ValueError):
        return GEOFENCE_MICRO_RADIUS_M
    return r if r > 0 else GEOFENCE_MICRO_RADIUS_M


def gcs_set_servo(channel: int, pwm: int) -> bool:
    result = gcs_post("/api/command", {"cmd": "set_servo", "params": {"channel": channel, "pwm": pwm}})
    return bool(result and result.get("ok"))


def gcs_payload_cfg() -> dict:
    """Payload servo channel + lock/release PWM, asked of the GCS.

    This file used to hardcode the channel (9) and both pulse widths
    (1410/1100), which made it a THIRD copy of numbers that config/*.yaml and
    the GCS UI already own. When the servos moved from AUX1/AUX2 to AUX5/AUX6
    on 2026-09-06, this was the copy that would have gone on commanding the
    dead pin - a delivery release that silently does nothing, with the receipt
    still written and the token still burned.

    GcsHub ships the whole `payload` block in every /api/state frame, so read
    it from there. Falls back to the config defaults if the GCS is unreachable;
    a failed lookup must not become a command on the wrong channel.
    """
    # The cached snapshot, so this never adds an HTTP round-trip of its own.
    # It runs on the release thread, but the servo pulse is already latency
    # sensitive enough without waiting on a 2 s timeout first.
    state = gcs_state()
    pl = (state or {}).get("payload") or {}
    if not pl:
        print("  ⚠️ [GCS] no payload config in /api/state - using defaults")
    return {
        "channel": int(pl.get("channel", 12)),
        "release_us": int(pl.get("release_us", 1410)),
        "lock_us": int(pl.get("lock_us", 1100)),
    }


# ══════════════════════════════════════════════════════════════════
#  SIGNED LOCAL RECEIPT (drone-side half of Dual Auditing)
# ══════════════════════════════════════════════════════════════════

# The old receipt here was a plain write_text() carrying the raw order token.
# Two of them were found torn on this aircraft's SD card (0 bytes and 299 zero
# bytes, 19 and 21 Sep) while their orders' tokens had been burned - see
# receipt_ledger.py. Every release is now a two-phase, fsynced, Ed25519-signed
# and hash-chained pair of records: release_intent BEFORE the actuator may
# move, then delivery / release_aborted / release_unconfirmed after.

_LEDGER = None
_LEDGER_LOCK = threading.Lock()


def get_ledger():
    """The process's ledger, created on first use (key generated once)."""
    global _LEDGER
    with _LEDGER_LOCK:
        if _LEDGER is None:
            key_path = Path(os.environ.get(
                "AERIX_RECEIPT_KEY", _HERE / "keys" / "drone_receipt_ed25519.pem"))
            root = Path(os.environ.get("AERIX_RECEIPT_DIR", _RECEIPTS_ROOT))
            _LEDGER = Ledger(root, DroneKey.load_or_create(key_path))
        return _LEDGER


def ledger_intent(order: ActiveOrder, **evidence) -> bool:
    """Commit the authorisation to release, durably, BEFORE anything moves.

    Fails closed: if the record cannot be made durable the release does not
    happen and the token is kept. A dead SD card costs one delivery attempt;
    it must not cost an unrecorded release."""
    try:
        rec = get_ledger().intent(order.order_id, order.token, **evidence)
    except Exception as exc:  # noqa: BLE001
        print(f"  ⛔ [LEDGER] could not commit the release intent ({exc}) - "
              "NOT releasing; token NOT spent.")
        return False
    print(f"  🧾 [LEDGER] release intent #{rec['body']['seq']} committed (fsynced, signed)")
    return True


def ledger_outcome(kind: str, order: ActiveOrder, **evidence) -> None:
    """Record how an intent ended. Never raises: the parcel's fate is already
    decided, and a dangling intent is closed as unconfirmed at next boot."""
    try:
        rec = get_ledger().outcome(kind, order.order_id, order.token, **evidence)
        print(f"  🧾 [LEDGER] {kind} #{rec['body']['seq']} committed")
    except Exception as exc:  # noqa: BLE001
        print(f"  ⚠️ [LEDGER] could not record {kind} ({exc}) - the open intent "
              "will be closed as release_unconfirmed at next start.")


# ══════════════════════════════════════════════════════════════════
#  BLE GATT SERVICE
# ══════════════════════════════════════════════════════════════════

class DroneService(Service):
    def __init__(self, order: ActiveOrder, order_file: Path | None):
        super().__init__(SERVICE_UUID, True)
        self.order = order
        self.order_file = order_file

        self.received_gps_lat = None
        self.received_gps_lng = None
        # nonce, the device it was issued to, auth, geofence window and ACK -
        # all of it one session, all of it reset together (see _new_session)
        self._new_session(_NOBODY)
        self.last_result = RESULT_PENDING
        self._last_geofence_log_ts = 0.0
        self._delivered = False         # true once this order's token is spent
        self._descent_in_progress = False  # precision drop running for this order
        self._drop_request_pending = False  # precision_drop request in flight
        self._rssi: RssiSampler | None = None

        print(f"[Pi 5] Order: {order.order_id}")
        print(f"[Pi 5] Order Target: ({order.target_lat}, {order.target_lng})  "
              f"micro<= {geofence_radius_m():g}m  "
              f"({TEMPORAL_READINGS_REQUIRED} consistent readings required)")

    def swap_order(self, order: ActiveOrder, order_file: Path | None) -> None:
        """Adopt a newly dispatched order without restarting the process.

        Called on the asyncio loop (same thread as every characteristic
        handler), so no handler can observe a half-swapped state. Any session
        in progress was authenticated against the OLD token and is dropped:
        the app must read a fresh nonce, exactly as after a reconnect."""
        self.order = order
        self.order_file = order_file
        self._new_session(_NOBODY)
        self.received_gps_lat = None
        self.received_gps_lng = None
        self.last_result = RESULT_PENDING
        self._delivered = False
        self._descent_in_progress = False
        self._drop_request_pending = False
        print(f"\n[ORDER] Hot-swapped to order {order.order_id} "
              f"target ({order.target_lat}, {order.target_lng}) - token refreshed")

    # ──────────────────────────────────────────────────────────────
    #  SESSION: one nonce, issued to one central, alive while it is used
    # ──────────────────────────────────────────────────────────────
    def _new_session(self, device) -> None:
        """Issue a fresh nonce to ``device`` and forget every earlier session.

        received_gps_lat/lng survive on purpose: the release thread snapshots
        them for the receipt, and a re-read nonce mid-release must not turn
        the receipt's coordinates into None."""
        self.current_nonce = os.urandom(16)
        self._nonce_device = device
        self._nonce_issued = _now()
        self._last_activity = self._nonce_issued
        self.is_authenticated = False
        self.gps_verified = False       # micro geofence passed (phone vs drone live GPS)
        self.gps_readings = []          # rolling window of (monotonic_ts, lat, lng) - anti-spoofing
        self.ack_hmac = bytes(20)       # Mutual auth ACK — empty until AUTH passes

    def _expire_session(self, why: str) -> None:
        print(f"  ⌛ SESSION EXPIRED ({why}) - nonce destroyed; the phone must "
              "read NONCE again. A captured session can no longer be replayed.")
        self._new_session(_NOBODY)

    def _owns_session(self, device) -> bool:
        """Is ``device`` the central this nonce was issued to, and is the
        session still alive? Expires a dead session as a side effect, and
        counts a call from the owner as activity."""
        if self._nonce_device is _NOBODY or device != self._nonce_device:
            return False
        now = _now()
        if self.is_authenticated:
            if now - self._last_activity > SESSION_IDLE_S:
                self._expire_session(f"silent {now - self._last_activity:.0f}s "
                                     f"> {SESSION_IDLE_S:g}s")
                return False
        elif now - self._nonce_issued > NONCE_TTL_S:
            self._expire_session(f"nonce unanswered {now - self._nonce_issued:.0f}s "
                                 f"> {NONCE_TTL_S:g}s")
            return False
        self._last_activity = now
        return True

    def _foreign(self, device, what: str) -> bool:
        """True (and logged) when ``device`` is some OTHER central than the
        session's. Such a request is dropped without touching any session
        state - otherwise a stranger could stamp AUTH_FAIL into the real
        phone's RESULT, or reset its geofence window, without authenticating."""
        if self._nonce_device is not _NOBODY and device == self._nonce_device:
            return False
        print(f"  🚫 {what} IGNORED: from {device or 'an unknown central'}, which "
              "does not hold this session's nonce (replay/relay or a second "
              "phone). Session untouched.")
        return True

    # ──────────────────────────────────────────────────────────────
    #  1. NONCE CHARACTERISTIC (READ ONLY) — Challenge
    # ──────────────────────────────────────────────────────────────
    @characteristic(NONCE_CHAR_UUID, CharacteristicFlags.READ)
    def nonce_characteristic(self, options):
        return self.on_nonce_read(_device_of(options))

    def on_nonce_read(self, device) -> bytes:
        # Every read starts a new session for the reader and kills any other.
        # (A stranger can therefore reset the phone's session - a nuisance on
        # the level of jamming, not a release; it cannot then USE the phone's
        # nonce, which is what binding prevents.)
        self._new_session(device)
        # A reconnect mid-descent (or after the release) must still be able
        # to learn the outcome once it re-authenticates.
        self.last_result = (RESULT_RELEASED if self._delivered
                            else RESULT_DESCENDING if self._descent_in_progress
                            else RESULT_PENDING)
        print(f"\n[BLE Read Nonce] New challenge for {device or 'unknown central'} "
              f"(valid {NONCE_TTL_S:g}s unanswered, {SESSION_IDLE_S:g}s idle)")
        return self.current_nonce

    # ──────────────────────────────────────────────────────────────
    #  2. AUTH CHARACTERISTIC (READ + WRITE) — Mutual Authentication
    # ──────────────────────────────────────────────────────────────
    @characteristic(AUTH_CHAR_UUID, CharacteristicFlags.READ | CharacteristicFlags.WRITE)
    def auth_characteristic(self, options):
        return self.on_ack_read(_device_of(options))

    def on_ack_read(self, device) -> bytes:
        # READ: Return the ACK HMAC so the app can verify drone identity -
        # only to the central that authenticated.
        if not self._owns_session(device) or not self.is_authenticated:
            print("[BLE Read Auth] ACK withheld - no live session for this central")
            return bytes(20)
        print("[BLE Read Auth] App reading ACK for mutual auth verification")
        return self.ack_hmac

    @auth_characteristic.setter
    def auth_characteristic(self, value, options):
        self.on_auth_write(bytes(value), _device_of(options))

    def on_auth_write(self, value: bytes, device) -> None:
        value = bytes(value)
        print(f"[BLE Write Auth] Received HMAC ({len(value)} bytes, fp {_fp(value)}) "
              f"from {device or 'unknown central'}")

        if self._foreign(device, "AUTH"):
            return
        if not self._owns_session(device):
            print("  ❌ AUTH REFUSED: this nonce has expired - read NONCE again.")
            return

        if self._delivered:
            print("  ⚠️ AUTH REJECTED: this order's token has already been used.")
            self.is_authenticated = False
            return

        # Calculate expected HMAC-SHA256(Nonce, OrderToken) truncated to 20 bytes
        expected_hmac = hmac.new(self.order.token, self.current_nonce, hashlib.sha256).digest()[:20]

        if hmac.compare_digest(value, expected_hmac):
            self.is_authenticated = True
            print("  ✅ AUTHENTICATION SUCCESSFUL! Drone unlocked.")

            # ── Generate Mutual Auth ACK ──
            # ACK = HMAC(Nonce + "ACK", OrderToken) truncated to 20 bytes
            ack_payload = self.current_nonce + b"ACK"
            self.ack_hmac = hmac.new(self.order.token, ack_payload, hashlib.sha256).digest()[:20]
            # Never log the ACK or an expected MAC: logs/ble is on the SD card.
            print("  🔑 Mutual Auth ACK generated.")
            self._start_rssi(device)
        else:
            self.is_authenticated = False
            self.ack_hmac = bytes(20)
            print("  ❌ AUTHENTICATION FAILED! Invalid HMAC signature.")
            # Two hex strings alone say nothing about WHY. Name the one cause
            # that guarantees this outcome, because it is also the one that
            # looks like a phone problem: on the bench token the app cannot
            # possibly match - it signs with the order's Firestore
            # deliveryToken, and this process is holding MY_SUPER_SECRET_KEY.
            # Everything downstream then fails at the auth gate, GPS writes
            # included, which reads as "the app never sent its coordinates".
            if self.order.is_dev:
                print("     ⚠️  CAUSE: this peripheral is on the HARDCODED BENCH "
                      "TOKEN (--dev), not a real order.")
                print("         The app signs with the order's Firestore "
                      "deliveryToken, so no app can ever authenticate here.")
                print("         Fix: place/dispatch an order in the app. The GCS "
                      "picks it up and this process hot-swaps the token within "
                      f"{ORDER_WATCH_S:.0f}s (no restart needed).")
            else:
                print("     Likely causes: the app holds a different order's "
                      "token, or this process's token is from an earlier flight.")

    # ──────────────────────────────────────────────────────────────
    #  3. GPS CHARACTERISTIC (WRITE ONLY) — Micro Geofence
    # ──────────────────────────────────────────────────────────────
    @characteristic(GPS_CHAR_UUID, CharacteristicFlags.WRITE)
    def gps_characteristic(self, options):
        return bytes()

    @gps_characteristic.setter
    def gps_characteristic(self, value, options):
        self.on_gps_write(bytes(value), _device_of(options))

    def on_gps_write(self, raw: bytes, device) -> None:
        raw = bytes(raw)
        print(f"\n[BLE Write GPS] Received GPS payload ({len(raw)} bytes)")

        if self._foreign(device, "GPS"):
            return
        if not self._owns_session(device) or not self.is_authenticated:
            # Say that the coordinates DID arrive. This gate is the top of the
            # GPS handler, so a failed handshake silently discards every fix
            # the phone streams - from the app it is indistinguishable from the
            # writes never leaving the phone, which is the wrong thing to go
            # and debug.
            print(f"  ⚠️ GPS REJECTED: payload arrived ({len(raw)} bytes) but the "
                  "connection never authenticated — the fix is NOT being read.")
            print("     Scroll up to the AUTHENTICATION FAILED line for the cause; "
                  "the phone's GPS is not the problem.")
            self.last_result = RESULT_AUTH_FAIL
            return

        try:
            # Parse the GPS payload: [gpsString bytes] + [0x00] + [16-byte HMAC]
            separator_index = raw.index(0x00)
            gps_bytes = raw[:separator_index]
            received_signature = raw[separator_index + 1:]

            gps_string = gps_bytes.decode("utf-8")
            print(f"  📍 GPS String: {gps_string}")

            # ── Verify GPS HMAC Signature ──
            # Expected: HMAC(nonce + gpsString, token) truncated to 16 bytes
            gps_hmac_payload = self.current_nonce + gps_bytes
            expected_gps_hmac = hmac.new(self.order.token, gps_hmac_payload, hashlib.sha256).digest()[:16]

            if not hmac.compare_digest(received_signature, expected_gps_hmac):
                print("  ❌ GPS SIGNATURE INVALID! Possible GPS data tampering.")
                self.gps_verified = False
                self.last_result = RESULT_SIGNATURE_FAIL
                return

            print("  🔐 GPS signature verified — data is authentic.")

            # ── Parse lat, lng ──
            parts = gps_string.split(",")
            lat = float(parts[0])
            lng = float(parts[1])
            self.received_gps_lat = lat
            self.received_gps_lng = lng
            # Signed, so it is the recipient's phone: let the navigator use it
            # to find them (the RSSI search starts from here).
            _post_async({"cmd": "ble_phone_fix", "params": {
                "order_id": self.order.order_id, "lat": lat, "lon": lng, "t": time.time()}})

            # Precision drop running (or the parcel already out): the fix above
            # still counts - it signs a retried DROP and feeds the locator - but
            # it must not re-judge the geofence. That would overwrite RESULT's
            # DESCENDING/RELEASED with OK/GEOFENCE_FAIL mid-descent, and a
            # phone polling RESULT would report a delivery that has not happened.
            if self._descent_in_progress or self._delivered:
                print("  🪂 GPS noted - precision drop in progress/complete, "
                      "RESULT left as " + self.last_result.decode())
                return

            # Pilot has the aircraft: judging the geofence now would only hand
            # the phone VERIFYING / GEOFENCE_OK for a delivery that cannot
            # happen. The fix still went to the locator above.
            if gcs_pilot_override():
                self.gps_verified = False
                self.last_result = RESULT_PILOT_OVERRIDE
                print("  ✋ GPS noted - the pilot has taken control, delivery "
                      "halted; geofence not evaluated.")
                return

            # ── Anti-spoofing: don't trust a single reading ──
            self._record_temporal_reading(lat, lng)
            have = len(self.gps_readings)
            if have < TEMPORAL_READINGS_REQUIRED:
                self.gps_verified = False
                # NOT GEOFENCE_FAIL. Nothing has been compared to anything yet,
                # so calling it a geofence failure made the app tell the
                # recipient to "get closer" while they were already 2.06 m away
                # (13:07 session) and only needed to stand still - and walking
                # would have reset this very window through the jump check.
                self.last_result = (RESULT_VERIFYING_PREFIX +
                                    f":{have}/{TEMPORAL_READINGS_REQUIRED}".encode())
                print(f"  ⏳ [Temporal Consistency] {have}/"
                      f"{TEMPORAL_READINGS_REQUIRED} consistent readings so far — "
                      f"hold still, nothing is being rejected.")
                return

            # Judge the median of the consistent window, not the newest fix:
            # one noisy reading should neither pass nor fail the recipient.
            lats = sorted(r[1] for r in self.gps_readings)
            lngs = sorted(r[2] for r in self.gps_readings)
            mid = len(lats) // 2
            self._check_geofences(lats[mid], lngs[mid])

        except Exception as e:
            print(f"  ❌ GPS PARSE ERROR: {e}")
            self.gps_verified = False
            self.last_result = RESULT_GEOFENCE_FAIL

    def _record_temporal_reading(self, lat: float, lng: float) -> None:
        """Anti-spoofing temporal consistency: a single GPS reading is cheap
        to fake (fake-GPS apps, a replayed value, a one-off multipath
        glitch). Require several consecutive readings, arriving within a
        short rolling window, that stay close to each other - a real person
        standing still naturally reports nearly the same spot each time; an
        attacker now has to fool the check repeatedly instead of once.

        An implausible jump between consecutive readings (further than a
        person could realistically move between BLE writes) is treated as
        a broken streak: the window restarts from this newest reading
        rather than being averaged in, so a spoofed/glitched outlier can't
        drag a otherwise-consistent streak across the line.
        """
        now = time.monotonic()
        self.gps_readings = [r for r in self.gps_readings if now - r[0] <= TEMPORAL_WINDOW_S]

        if self.gps_readings:
            _, prev_lat, prev_lng = self.gps_readings[-1]
            jump = haversine_distance(prev_lat, prev_lng, lat, lng)
            if jump > TEMPORAL_MAX_JUMP_M:
                print(f"  ⚠️ [Temporal Consistency] implausible jump ({jump:.1f}m since "
                      f"the last reading) — treating as spoofed/glitched, restarting "
                      f"the consistency window.")
                self.gps_readings = []

        self.gps_readings.append((now, lat, lng))

    def _check_geofences(self, phone_lat: float, phone_lng: float) -> None:
        # ── Micro Geofence — phone vs the DRONE's own live GPS ──
        # Only reached once TEMPORAL_READINGS_REQUIRED consistent readings
        # are in hand (see gps_characteristic.setter).
        drone_pos, reason = get_drone_live_gps()
        if drone_pos is None:
            self.gps_verified = False
            # NOT GEOFENCE_FAIL: the phone's fix was fine and verified, the
            # drone simply cannot say where it is, so there is nothing the
            # recipient can do by moving. Token is not spent; the app retries.
            self.last_result = RESULT_NO_DRONE_FIX
            print(f"  ❌ GEOFENCE UNDECIDABLE — {reason}")
            print("     (the phone's fix arrived and verified; the drone's own "
                  "position is what is missing)")
            print("Handshake Unsuccessful")
            return

        drone_lat, drone_lng = drone_pos
        radius = geofence_radius_m()
        micro_distance = haversine_distance(phone_lat, phone_lng, drone_lat, drone_lng)
        print(f"  📏 [Micro Geofence] phone (median of window) vs drone's live "
              f"position: {micro_distance:.2f}m (max {radius:g}m)")

        if micro_distance <= radius:
            self.gps_verified = True
            # GEOFENCE_OK, never a bare OK: this says the gate passed, not that
            # the parcel has gone. See the RESULT_OK comment above.
            self.last_result = RESULT_GEOFENCE_OK
            print(f"  ✅ GEOFENCE PASSED! Drone is directly over the recipient.")
            print("Handshake Successful")
            return

        # Geofence failed. There is no autonomous relocation any more (removed
        # 2026-09-27), so nothing on the aircraft will close this gap: say so
        # rather than letting the 60 s timeout that follows look like a fault.
        self.gps_verified = False
        self.last_result = RESULT_GEOFENCE_FAIL
        print("Handshake Unsuccessful")
        now = time.monotonic()
        if now - self._last_geofence_log_ts >= GEOFENCE_LOG_COOLDOWN_S:
            self._last_geofence_log_ts = now
            print(f"  🔄 GEOFENCE FAILED — you are {micro_distance:.1f}m from the "
                  f"aircraft, limit {radius:g}m.")
            print(f"     The drone will NOT move to you. Walk to within "
                  f"{radius:g}m of it and hold still for "
                  f"{TEMPORAL_READINGS_REQUIRED} readings.")
        else:
            print(f"  🔄 GEOFENCE still failing ({micro_distance:.1f}m).")

    # ──────────────────────────────────────────────────────────────
    #  4. DROP CHARACTERISTIC (WRITE ONLY) — Release Parcel
    # ──────────────────────────────────────────────────────────────
    @characteristic(DROP_CHAR_UUID, CharacteristicFlags.WRITE)
    def drop_characteristic(self, options):
        return bytes()

    @drop_characteristic.setter
    def drop_characteristic(self, value, options):
        self.on_drop_write(bytes(value), _device_of(options))

    def on_drop_write(self, value: bytes, device) -> None:
        value = bytes(value)
        print(f"\n[BLE Write Drop] Received Drop Command ({len(value)} bytes, "
              f"fp {_fp(value)})")

        # First, before the idempotency answers below: a stranger learns
        # nothing, and changes nothing, by writing DROP.
        if self._foreign(device, "DROP"):
            return
        live = self._owns_session(device)

        # ── Idempotency: this token has already been spent — never re-fire
        # the release/receipt/invalidation for a duplicate or replayed DROP
        # write (a stray BLE retransmit can carry an identical signature). ──
        if self._delivered:
            print("  ⚠️ DROP IGNORED: this order has already been delivered.")
            return
        if self._descent_in_progress:
            print("  ⚠️ DROP IGNORED: precision drop already in progress.")
            return
        if self._drop_request_pending:
            # A request is already with the navigator. Re-firing it on every
            # BLE retry would queue duplicate drops behind one set of gates.
            print("  ⚠️ DROP IGNORED: a drop request is already in flight "
                  f"(RESULT stays {self.last_result.decode()}).")
            return

        # ── Security Gate 1: Must be authenticated, on a live session ──
        if not live or not self.is_authenticated:
            print("  ⚠️ DROP REJECTED: Connection is not authenticated!")
            self.last_result = RESULT_AUTH_FAIL
            return

        # ── Security Gate 2: Must have passed the micro geofence ──
        if not self.gps_verified:
            print("  ⚠️ DROP REJECTED: Geofence verification not passed!")
            print(f"     Micro geofence: FAIL")
            self.last_result = RESULT_GEOFENCE_FAIL
            return

        # ── Security Gate 4: Verify DROP HMAC signature ──
        # Enhanced DROP HMAC: HMAC(nonce + gpsString + "DROP", token) truncated to 20 bytes
        gps_string = f"{self.received_gps_lat:.8f},{self.received_gps_lng:.8f}"
        gps_bytes = gps_string.encode("utf-8")
        drop_payload = self.current_nonce + gps_bytes + b"DROP"
        expected_drop_hmac = hmac.new(self.order.token, drop_payload, hashlib.sha256).digest()[:20]

        if not hmac.compare_digest(bytes(value), expected_drop_hmac):
            print("  ❌ DROP REJECTED: Invalid Drop HMAC payload.")
            self.last_result = RESULT_SIGNATURE_FAIL
            return

        # ── Gate 5: the drone must have locked onto a person below ──
        # Checked last so a bad signature is still reported as one. Nothing
        # is spent here: the app retries the same DROP until the navigator
        # opens the window (or the flight gives up and returns home).
        # Checked before the person-lock gate because a takeover SHUTS that
        # window: without this, the refusal below would blame "no person
        # locked" for a mission the pilot has already ended.
        if gcs_pilot_override():
            print("  ✋ DROP REFUSED: the pilot has taken control - delivery halted "
                  "by the navigator; token NOT spent. Resume from the GCS to retry.")
            self.last_result = RESULT_PILOT_OVERRIDE
            return

        if not gcs_handshake_open():
            print("  ⏳ DROP DEFERRED: no person locked at the drop point yet "
                  "(or the window closed) - token NOT spent, retry.")
            self.last_result = RESULT_NOT_READY
            return

        # ── Gate 6: the token must belong to the order being flown ──
        other = gcs_flying_other_order(self.order.order_id)
        if other is not None:
            print(f"  ⏳ DROP DEFERRED: the GCS is flying order {other}, this token is "
                  f"for {self.order.order_id} - token NOT spent.")
            self.last_result = RESULT_NOT_READY
            return

        print("\n" + "=" * 60)
        print("  🪂 ALL SECURITY GATES PASSED!")
        print(f"     ✅ Gate 1: HMAC Authentication   — PASSED")
        print(f"     ✅ Gate 2: Micro Geofence ({geofence_radius_m():g}m)  — PASSED")
        print(f"     ✅ Gate 3: Drop HMAC Signature   — PASSED")
        print(f"     ✅ Gate 4: Person locked below   — PASSED")
        print(f"     ✅ Gate 5: Order matches flight  — PASSED")
        print("=" * 60 + "\n")

        # Ask the navigator to fly the precision drop (TF02-Pro descent to
        # 1 m beside the recipient). It answers one of three ways:
        #   ok                -> it is flying it; we watch for the release
        #   data.mode=legacy  -> disabled / on the ground: release directly,
        #                        exactly as before the precision drop existed
        #   anything else     -> refused (no TF02 reading, window closed...):
        #                        NOT_READY, token kept, the app may retry
        # Asking the navigator to fly the drop is an HTTP round-trip, so it goes
        # to a worker thread: this handler must return promptly or the BLE link
        # it is answering on stalls with it. The phone is already in a DROP
        # retry loop, so NOT_READY now and the real verdict on the next poll is
        # exactly the flow the protocol was built for - and the token stays
        # unspent either way.
        self._drop_request_pending = True
        self.last_result = RESULT_NOT_READY
        print("  ⏳ DROP ACCEPTED FOR DISPATCH: asking the navigator off-thread; "
              "read RESULT again for the verdict (token NOT spent).")
        threading.Thread(target=self._request_drop_worker,
                         args=(self.order, self.order_file,
                               self.received_gps_lat, self.received_gps_lng),
                         kwargs={"evidence": self._drop_evidence(device)},
                         daemon=True, name="drop-request").start()

    def _drop_evidence(self, device) -> dict:
        """What the release was authorised ON, for the signed intent record.

        Captured here, on the BLE thread, at the moment every gate passed -
        not later on the worker, where the session may already have moved on.
        Nothing in it can authenticate anything: the nonce and the central
        are hashed, the token is never present."""
        ev = {
            "phoneLat": self.received_gps_lat, "phoneLng": self.received_gps_lng,
            "geofenceM": geofence_radius_m(),
            "nonceHash": hashlib.sha256(self.current_nonce).hexdigest()[:16],
            "centralHash": (hashlib.sha256(str(device).encode()).hexdigest()[:16]
                            if device else None),
        }
        pos, _ = get_drone_live_gps()
        if pos is not None and self.received_gps_lat is not None:
            ev["droneLat"], ev["droneLng"] = pos
            ev["distM"] = round(haversine_distance(
                pos[0], pos[1], self.received_gps_lat, self.received_gps_lng), 2)
        return ev

    def _request_drop_worker(self, order: ActiveOrder, order_file: Path | None,
                             lat: float, lng: float, evidence: dict | None = None) -> None:
        """Ask the navigator for the precision drop, off the BLE event loop.

        Answers one of three ways:
          ok                -> it is flying it; we watch for the release
          data.mode=legacy  -> disabled / on the ground: release directly,
                               exactly as before the precision drop existed
          anything else     -> refused (no TF02 reading, window closed...):
                               NOT_READY, token kept, the app may retry

        Write-ahead: the signed release_intent is made durable BEFORE the
        navigator is asked, because once it says ok the navigator fires the
        servo itself, in another process, and a brownout there would take this
        one down too. Every way out of here then closes that intent - delivery
        (in the watch / the legacy release), release_aborted (refused, nothing
        moved), or release_unconfirmed at next boot if nothing got to say.
        """
        try:
            if not ledger_intent(order, **(evidence or {})):
                self.last_result = RESULT_NOT_READY      # fail closed
                return
            try:
                resp = request_precision_drop(order.order_id)
            except Exception as exc:  # noqa: BLE001
                resp = None
                print(f"  ⚠️ [DROP] precision_drop request raised: {exc}")

            # The order may have been hot-swapped while the request was in
            # flight; this verdict belongs to the order that asked for it.
            if self.order is not order:
                print("  ⚠️ [DROP] order swapped during the request - verdict dropped")
                # If the navigator accepted, it may fly the drop unwatched: we
                # cannot say the parcel stayed aboard, so do not claim it.
                ledger_outcome(KIND_UNCONFIRMED if resp and resp.get("ok") else KIND_ABORTED,
                               order, why="order swapped during the request")
                return

            if resp is None:
                print("  ⏳ DROP DEFERRED: GCS unreachable - token NOT spent, retry.")
                ledger_outcome(KIND_ABORTED, order, why="GCS unreachable")
                self.last_result = RESULT_NOT_READY
                return
            data = resp.get("data") or {}
            msg = str(resp.get("message", ""))
            if resp.get("ok"):
                print(f"  🪂 PRECISION DROP ACCEPTED: {msg} - drone moving aside and "
                      f"descending to 1 m (TF02-Pro).")
                self._descent_in_progress = True
                self.last_result = RESULT_DESCENDING
                self._watch_precision_drop(order, order_file, lat, lng)
                return
            if data.get("mode") == "legacy" or msg.startswith("no such service"):
                # A real order must never release on the ground. On 2026-09-27
                # a phone still connected after a failed flight retried DROP
                # once the aircraft had landed; the navigator answered
                # "disarmed - bench release", and three parcels were marked
                # DELIVERED at the launch point with their tokens burned.
                # Two independent signals, either one refuses: the navigator's
                # own flag, and the telemetry (fails closed). Only the bench
                # token may release on the ground.
                if not order.is_dev and (data.get("on_ground") or not gcs_armed()):
                    print(f"  ⛔ DROP REFUSED: aircraft is on the ground ({msg}) - no "
                          "release for a real order; token NOT spent.")
                    ledger_outcome(KIND_ABORTED, order, why="on the ground", navigator=msg)
                    self.last_result = RESULT_NOT_READY
                    return
                print(f"  🪂 DIRECT RELEASE ({msg or 'navigator has no precision drop'}) - "
                      "TRIGGERING SERVO/SOLENOID RELEASE...")
                self._legacy_release()
                return
            print(f"  ⏳ DROP DEFERRED: navigator refused the precision drop: {msg} "
                  "- token NOT spent, retry.")
            ledger_outcome(KIND_ABORTED, order, why="navigator refused", navigator=msg)
            self.last_result = RESULT_NOT_READY
        finally:
            # Cleared last: while it is set, a retried DROP is answered from
            # last_result instead of firing a second request at the navigator.
            self._drop_request_pending = False

    def _legacy_release(self) -> None:
        """The pre-2026-09-26 release: servo straight from the hold height."""
        self._delivered = True
        # RELEASED, not OK: the same code the precision drop reports, so
        # "the parcel is gone" has exactly one representation on the wire.
        self.last_result = RESULT_RELEASED

        # Tell drone_stack the handshake is done - NavigationNode cuts its
        # drop-point hold short a few seconds after this instead of waiting
        # out the full hover timeout (see gcs/hub.py's
        # _on_ble_delivery_result). Fired here, at the security-gate
        # decision, not after the servo pulse below: the release gate having
        # been verified (auth + geofence + DROP HMAC) is the "delivery
        # complete" signal regardless of payload-servo hardware state.
        # Threaded: this runs on the drop-request worker, but the servo pulse
        # below must not queue behind an HTTP timeout to a GCS that may already
        # be struggling - the release is the time-critical half.
        _post_async({
            "cmd": "ble_delivery_result",
            "params": {"order_id": self.order.order_id, "success": True},
        })

        # Run the servo pulse + receipt + token invalidation off the asyncio
        # loop (it includes a multi-second wait for the servo to clear) so it
        # never blocks the BLE/D-Bus event loop.
        threading.Thread(target=self._release_and_finalize, daemon=True).start()

    def _watch_precision_drop(self, order: ActiveOrder, order_file: Path | None,
                              lat: float, lng: float) -> None:
        """Follow the navigator's drop until the parcel is away or it gives up.

        The navigator fires the servo and publishes the delivery result itself
        (Firebase marks the order DELIVERED from that), so all that is left
        here is the drone-side receipt and the single-use token. Neither
        happens unless /api/state shows THIS order's drop past the release.
        """
        started = time.monotonic()
        stage, m = "", {}
        while time.monotonic() - started < DROP_WATCH_MAX_S:
            time.sleep(DROP_POLL_S)
            if self.order is not order:
                print("  ⚠️ [DROP] order swapped mid-drop - watch abandoned")
                ledger_outcome(KIND_UNCONFIRMED, order, why="order swapped mid-drop",
                               dropStage=stage)
                return
            m = gcs_mission_state()
            if m is None or m.get("drop_order_id") != order.order_id:
                continue
            new = str(m.get("drop_stage") or "")
            if new != stage:
                stage = new
                print(f"  🪂 [DROP] stage: {stage or '-'}  "
                      f"TF02 {m.get('drop_agl_m', 0)} m  {m.get('drop_note', '')}")
            if stage in ("release", "climb", "done"):
                self._delivered = True
                self._descent_in_progress = False
                self.last_result = RESULT_RELEASED
                print("[HARDWARE] Parcel released at the release height (navigator).")
                ledger_outcome(KIND_DELIVERY, order, dropStage=stage,
                               aglM=m.get("drop_agl_m"), note=m.get("drop_note", ""))
                if order_file is not None:
                    _invalidate_order_file(order_file)
                print("[STATE] Delivery complete — order token expired; the drone "
                      "climbs back to 5 m and returns home.\n")
                return
            if stage == "aborted":
                self._descent_in_progress = False
                # A pilot takeover is not "retry the drop": the navigator has
                # stood down and nothing will reopen the window by itself.
                self.last_result = (RESULT_PILOT_OVERRIDE if gcs_pilot_override()
                                    else RESULT_NOT_READY)
                print(f"  ⚠️ [DROP] precision drop ABORTED ({m.get('drop_note', '')}) - "
                      "parcel still aboard, token NOT spent.")
                ledger_outcome(KIND_ABORTED, order, why="navigator aborted the drop",
                               dropStage=stage, aglM=m.get("drop_agl_m"),
                               note=m.get("drop_note", ""))
                return
        # Lost track of it. If the parcel did go, the navigator's own state
        # says "done" and a retried DROP is answered idempotently with it.
        self._descent_in_progress = False
        self.last_result = RESULT_NOT_READY
        print(f"  ⚠️ [DROP] no release seen in {DROP_WATCH_MAX_S:.0f}s - token NOT spent.")
        # Not "aborted": we stopped looking, we did not see it stay aboard.
        ledger_outcome(KIND_UNCONFIRMED, order, why="no release seen in the watch window",
                       dropStage=stage)

    def _start_rssi(self, device) -> None:
        """One sampler per authenticated connection, on THAT connection."""
        path = device or ""
        if self._rssi is not None and self._rssi.is_alive():
            if self._rssi.device_path == path:
                return
            self._rssi.stop_evt.set()
        self._rssi = RssiSampler(self.order.order_id, path)
        self._rssi.start()

    def _release_and_finalize(self) -> None:
        """Hardware release + dual-audit receipt + token expiry.

        Runs on a background thread (see drop_characteristic.setter) so the
        multi-second servo settle time never blocks the BLE event loop.
        """
        # Snapshot first: _order_watch may swap self.order for the NEXT
        # dispatch while this thread sleeps, and the receipt + token expiry
        # must stay with the order that was actually delivered.
        order, order_file = self.order, self.order_file
        # Channel and both pulse widths come from the GCS (config is the
        # single source of truth) - never hardcoded here. See gcs_payload_cfg.
        _cfg = gcs_payload_cfg()
        out_ch = _cfg["channel"]
        open_pwm = _cfg["release_us"]
        close_pwm = _cfg["lock_us"]

        # ── COMMIT THE AUDIT STATE BEFORE TOUCHING THE HARDWARE ──
        # Order matters, and not for tidiness: the servo's inrush current can
        # brown out the Pi. If that happens after the parcel has gone but
        # before these two lines run, the delivery is missing from the audit
        # trail AND the single-use token survives to be replayed - the worst
        # combination available. Committing first can at most over-record a
        # release that failed, which the GCS state and the phone's own receipt
        # both contradict; committing after can silently lose a real one.
        # The signed release_intent was committed (fsynced) by
        # _request_drop_worker before it got here; the token goes next, then
        # the servo, then the outcome. A brownout anywhere after the intent
        # leaves it open, and the next start closes it as release_unconfirmed
        # - recorded, never silently lost.
        if order_file is not None:
            _invalidate_order_file(order_file)
        print("[STATE] Delivery state committed — firing servo.\n")

        opened = gcs_set_servo(out_ch, open_pwm)
        if opened:
            print(f"[HARDWARE] Servo OPENED (PWM {open_pwm}) -> Parcel Released!")
        else:
            print("  ❌ [HARDWARE ERROR] Failed to send open command.")
        # DO_SET_SERVO accepted is not proof the pin moved (CLAUDE.md §6c), so
        # even "delivery" here means "the FC accepted the release" - the
        # evidence says which channel and pulse, so an auditor can tell.
        ledger_outcome(KIND_DELIVERY if opened else KIND_UNCONFIRMED, order,
                       path="legacy", channel=out_ch, pwm=open_pwm, fcAccepted=opened)

        print("[HARDWARE] Waiting 2 seconds before resetting servo...")
        time.sleep(2)

        if gcs_set_servo(out_ch, close_pwm):
            print(f"[HARDWARE] Servo CLOSED (PWM {close_pwm}) -> Ready for next drop.")
        else:
            print("  ❌ [HARDWARE ERROR] Failed to send close/reset command.")

        print("[STATE] Delivery complete — order token expired, ready for next flight.\n")

    # ──────────────────────────────────────────────────────────────
    #  5. RESULT CHARACTERISTIC (READ ONLY) — Outcome of last DROP attempt
    # ──────────────────────────────────────────────────────────────
    @characteristic(RESULT_CHAR_UUID, CharacteristicFlags.READ)
    def result_characteristic(self, options):
        return self.on_result_read(_device_of(options))

    def on_result_read(self, device) -> bytes:
        # Access control: only the authenticated peer learns the real outcome.
        # Anyone else gets AUTH_FAIL - exactly what it would legitimately earn
        # by attempting a drop without a handshake - so the answer reveals
        # nothing. A poll from the owner keeps its session alive.
        if not self._owns_session(device) or not self.is_authenticated:
            print("[BLE Read Result] Unauthenticated read — returning AUTH_FAIL "
                  "(real outcome withheld).")
            return RESULT_AUTH_FAIL
        print(f"[BLE Read Result] App reading last DROP outcome: {self.last_result.decode()}")
        return self.last_result

    # ──────────────────────────────────────────────────────────────
    #  6. RESULT_SIGNED (READ ONLY) — the outcome, authenticated to the phone
    # ──────────────────────────────────────────────────────────────
    # RESULT alone is plain bytes: a fake drone (or a relay in the middle)
    # can answer RELEASED, and an app that trusts it marks a parcel delivered
    # that is still aboard. Nothing but the token holder can compute this MAC,
    # and it is bound to this session's nonce, so an old RELEASED from an
    # earlier session cannot be replayed into a new one. One read returns code
    # and MAC together, so they can never come from two different moments.
    @characteristic(RESULT_SIGNED_CHAR_UUID, CharacteristicFlags.READ)
    def result_signed_characteristic(self, options):
        return self.on_result_signed_read(_device_of(options))

    def on_result_signed_read(self, device) -> bytes:
        code = self.on_result_read(device)
        if code == RESULT_AUTH_FAIL and not (self._owns_session(device)
                                             and self.is_authenticated):
            return code + b"\x00" + bytes(16)   # no session: nothing to sign with
        tag = hmac.new(self.order.token, self.current_nonce + b"RESULT" + code,
                       hashlib.sha256).digest()[:16]
        return code + b"\x00" + tag


# ══════════════════════════════════════════════════════════════════
#  MAIN — Start BLE Server
# ══════════════════════════════════════════════════════════════════

# ══════════════════════════════════════════════════════════════════
#  ORDER HOT-RELOAD
# ══════════════════════════════════════════════════════════════════
#
# The token used to be pulled ONCE, at boot (ble_handshake_start.sh). An order
# dispatched after boot - i.e. every real flight - was then authenticated
# against the previous order's token, and the app got AUTHENTICATION FAILED
# with nothing to say why (2026-09-24 13:40 flight: GCS on one order, BLE on
# the 13:38 one). Now the peripheral follows the order the GCS is flying.

ORDER_WATCH_S = 3.0
ORDER_PULL_RETRY_S = 30.0
_DEFAULT_CRED = "/home/pi/drone_stack/config/firebase-service-account.json"


def _gcs_order_id() -> str | None:
    """The order the GCS currently holds, or None. Deliberately quiet (unlike
    gcs_get): the GCS restarts often and a 3 s poll must not flood the log."""
    try:
        with urllib.request.urlopen(f"{GCS_BASE_URL}/api/state",
                                    timeout=GCS_HTTP_TIMEOUT_S) as response:
            state = json.loads(response.read().decode())
    except (urllib.error.URLError, TimeoutError, ValueError, OSError):
        return None
    delivery = state.get("delivery") if isinstance(state, dict) else None
    oid = delivery.get("order_id") if isinstance(delivery, dict) else None
    return str(oid) if oid else None


def _spent_order_ids(order_file: Path) -> set[str]:
    """Every order whose token was already used for a DROP on this aircraft,
    from the active_order.used_<ts>.json files _invalidate_order_file leaves."""
    spent: set[str] = set()
    where = order_store.spent_dir(order_file)
    for path in where.glob(f"{order_file.stem}.used_*{order_file.suffix}"):
        try:
            spent.add(json.loads(path.read_text(encoding="utf-8"))["orderId"])
        except (OSError, ValueError, KeyError):
            continue
    return spent


def _pull_order_sync(cred: str, order_id: str) -> Path | None:
    """firebase_sync.pull_order, off the event loop. Returns the file it wrote."""
    try:
        import firebase_sync
        firebase_sync._init_app(cred)
        if firebase_sync.pull_order(order_id) != 0:
            return None
        return firebase_sync._ACTIVE_ORDER_PATH
    except Exception as exc:  # firebase/network errors must never kill BLE
        print(f"  ❌ [ORDER] pull of {order_id} failed: {exc}")
        return None


async def _order_watch(service: "DroneService", order_file: Path, cred: str) -> None:
    spent = _spent_order_ids(order_file)
    failed_at: dict[str, float] = {}
    print(f"[ORDER] Watching GCS for new orders every {ORDER_WATCH_S:.0f}s "
          f"({len(spent)} spent order id(s) on record)")
    while True:
        await asyncio.sleep(ORDER_WATCH_S)
        if service._delivered:
            spent.add(service.order.order_id)
        oid = await asyncio.to_thread(_gcs_order_id)
        if not oid or oid == service.order.order_id or oid in spent:
            continue
        if time.monotonic() - failed_at.get(oid, -math.inf) < ORDER_PULL_RETRY_S:
            continue
        print(f"\n[ORDER] GCS is on order {oid}, BLE holds {service.order.order_id} "
              f"- pulling the new token")
        path = await asyncio.to_thread(_pull_order_sync, cred, oid)
        try:
            new = ActiveOrder.load(path) if path is not None else None
        except (OSError, ValueError, KeyError) as exc:
            print(f"  ❌ [ORDER] pulled file unreadable: {exc}")
            new = None
        if new is None or new.order_id != oid:
            failed_at[oid] = time.monotonic()
            continue
        service.swap_order(new, path)


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Drone BLE handshake peripheral")
    parser.add_argument(
        "--order-file", type=Path, default=order_store.default_order_path(),
        help="path to the active_order.json written by firebase_sync.py pull-order "
             "(default: RAM - /run/aerix-ble, or $AERIX_ORDER_DIR; see order_store.py)",
    )
    parser.add_argument(
        "--max-order-age-s",
        type=float,
        default=float(os.environ.get("AERIX_BLE_MAX_ORDER_AGE_S", 6 * 3600)),
        help="refuse an active_order.json older than this many seconds "
             "(0 disables the check). A deliveryToken is single-use and issued "
             "per flight, so a stale file means guaranteed HMAC failures.",
    )
    parser.add_argument(
        "--cred", default=os.environ.get("AERIX_FIREBASE_CRED", _DEFAULT_CRED),
        help="Firebase service-account JSON used to pull a newly dispatched "
             "order's token at runtime (see _order_watch)",
    )
    parser.add_argument(
        "--dev", action="store_true",
        help="bench-testing fallback: use a hardcoded token/target instead of "
             "requiring active_order.json (INSECURE — do not use for a real delivery)",
    )
    return parser.parse_args(argv)


class _DailyTee:
    """Copy everything written to a stream into logs/ble/ble-YYYYMMDD.log.

    journald on this Pi is volatile, so the peripheral's only record of a
    handshake (auth, each GPS write, geofence distance, DROP verdict) used to
    die with the next power cycle - which is exactly how the 27 Sep 11:12-11:25
    flights' "why did it never drop" trail was lost to an 11:28 reboot. The
    file is the durable copy; stdout still goes to the journal as before.
    Logging must never take the handshake down, so any OSError disables it.
    """

    def __init__(self, stream, log_dir: Path):
        self._stream = stream
        self._dir = log_dir
        self._day = ""
        self._fh = None
        self._at_line_start = True
        self._lock = threading.Lock()

    def _file(self):
        day = time.strftime("%Y%m%d")
        if day != self._day or self._fh is None:
            if self._fh is not None:
                self._fh.close()
            self._dir.mkdir(parents=True, exist_ok=True)
            path = self._dir / f"ble-{day}.log"
            self._fh = open(path, "a", encoding="utf-8", buffering=1)
            owner = self._dir.parent.stat()          # root service, pi's tree:
            for p in (self._dir, path):              # keep it pi-owned
                try:
                    os.chown(p, owner.st_uid, owner.st_gid)
                except OSError:
                    pass
            self._day = day
        return self._fh

    def write(self, s):
        n = self._stream.write(s)
        if self._dir is None or not s:
            return n
        with self._lock:
            try:
                fh = self._file()
                for part in s.splitlines(keepends=True):
                    if self._at_line_start:
                        t = time.time()
                        fh.write(time.strftime("%H:%M:%S", time.localtime(t))
                                 + f".{int(t * 1000) % 1000:03d} ")
                    fh.write(part)
                    self._at_line_start = part.endswith("\n")
            except OSError as exc:
                self._dir = None
                self._stream.write(f"  ⚠️  [LOG] ble file log disabled: {exc}\n")
        return n

    def flush(self):
        self._stream.flush()

    def __getattr__(self, name):
        return getattr(self._stream, name)


def _persist_output() -> None:
    log_dir = Path(os.environ.get("AERIX_BLE_LOG_DIR",
                                  _HERE.parent.parent / "logs" / "ble"))
    sys.stdout = _DailyTee(sys.stdout, log_dir)
    sys.stderr = _DailyTee(sys.stderr, log_dir)
    print(f"  📝 [LOG] handshake log -> {log_dir}/ble-{time.strftime('%Y%m%d')}.log")


async def _onboard_adapter(bus):
    """(Adapter, "hciN") for the Pi's built-in controller, or (None, None).

    Picked by sysfs bus path, not by number: hciN follows registration order,
    so a USB dongle can take hci0 if it attaches first. (None, None) makes the
    caller fall back to bluez_peripheral's own first-adapter choice - exactly
    the pre-2026-10-10 behaviour - so a resolution failure can never leave the
    handshake worse off than it was.
    """
    base = "/sys/class/bluetooth"
    try:
        names = sorted(n for n in os.listdir(base) if n.startswith("hci") and ":" not in n)
    except OSError:
        return None, None
    for name in names:
        if "/usb" in os.path.realpath(os.path.join(base, name)):
            continue
        path = f"/org/bluez/{name}"
        try:
            intro = await bus.introspect("org.bluez", path)
            return Adapter(bus.get_proxy_object("org.bluez", path, intro)), name
        except Exception as exc:  # noqa: BLE001
            print(f"  ⚠️ [BLE] on-board {name} not usable via BlueZ ({exc}); "
                  "falling back to the first adapter")
            return None, None
    return None, None


async def main():
    _persist_output()
    args = _parse_args()

    # Close what the last run could not: an intent with no outcome means the
    # process died between authorising a release and recording how it ended
    # (the brownout-at-servo-fire case). It becomes release_unconfirmed - a
    # signed admission that the drone does not know - never a silent gap.
    try:
        for rec in get_ledger().recover():
            print(f"  🧾 [LEDGER] closed dangling intent for order "
                  f"{rec['body']['orderId']} as release_unconfirmed")
    except Exception as exc:  # noqa: BLE001
        print(f"  ⚠️ [LEDGER] recovery failed ({exc}); releases will fail closed "
              "until the ledger is writable.")

    # A delivery token is single-use and issued per flight, so an order file is
    # only meaningful for as long as the flight it was pulled for. Age matters
    # because the pull at boot is allowed to fail quietly ("no DISPATCHED order
    # available - this is normal when idle"), and without this check the
    # peripheral would then fall back to whatever active_order.json happened to
    # be left on disk from a previous day and advertise it as "SECURE per-order
    # mode". That is exactly what happened on 2026-08-30: a token pulled on
    # 08-27 was served to the app three days later, and every handshake failed
    # HMAC authentication with no indication of why.
    #
    # A stale file is therefore treated as no file at all.
    stale_age = None
    if args.order_file.exists() and args.max_order_age_s > 0:
        age = time.time() - args.order_file.stat().st_mtime
        if age > args.max_order_age_s:
            stale_age = age

    if stale_age is not None:
        hours = stale_age / 3600.0
        print(f"❌ Order file {args.order_file} is {hours:.1f} h old "
              f"(limit {args.max_order_age_s / 3600.0:.1f} h) - REFUSING it.")
        print("   Its deliveryToken is single-use and almost certainly no longer")
        print("   the one the app holds, so every handshake would fail HMAC auth.")
        print("   Dispatch an order in the app, then restart this service:")
        print("     sudo systemctl restart aerix-ble")
        if not args.dev:
            sys.exit(1)
        print("   --dev given: coming up on the bench token instead so the")
        print("   peripheral still advertises. NOT a real delivery.")

    if args.order_file.exists() and stale_age is None:
        order = ActiveOrder.load(args.order_file)
        order_file = args.order_file
    elif args.dev:
        print("  ⚠️  --dev mode: using hardcoded bench-testing token/target. "
              "This is NOT a per-order dynamic token — do not use for a real delivery.")
        order = ActiveOrder.dev_fallback()
        order_file = None
    else:
        print(f"❌ No active order file at {args.order_file}.")
        print("   Run 'python3 firebase_sync.py pull-order' first (needs a DISPATCHED")
        print("   order in Firestore + GOOGLE_APPLICATION_CREDENTIALS set), or pass --dev")
        print("   for bench testing without Firestore.")
        sys.exit(1)

    # Start polling /api/state before the first phone can connect: every
    # characteristic handler reads this cache instead of doing its own HTTP, so
    # that no BLE write is ever answered from inside a blocking socket call.
    global _GCS_CACHE
    _GCS_CACHE = GcsStateCache()
    _GCS_CACHE.start()

    bus = await get_message_bus()

    # Pin the handshake to the ON-BOARD controller. bluez_peripheral otherwise
    # takes whichever adapter BlueZ lists first, and since the MedBox bridge
    # (aerix-payload.service) put a USB dongle beside it that order is no
    # longer guaranteed. The bridge owns the dongle; this owns the on-board one.
    hs_adapter, hs_hci = await _onboard_adapter(bus)
    print(f"[BLE] handshake adapter: {hs_hci or 'first available (on-board not resolved)'}")

    service = DroneService(order, order_file)
    await service.register(bus, adapter=hs_adapter)

    # BLE Advertisement (matches target UUID scanned by app)
    advertisement = Advertisement(
        localName="Target Drone Detected",
        serviceUUIDs=[SERVICE_UUID],
        appearance=0,
        timeout=0,
    )
    await advertisement.register(bus, hs_adapter)

    agent = NoIoAgent()
    await agent.register(bus)

    print("\n" + "=" * 60)
    print(" 🛸 RASPBERRY PI 5 DRONE BLE SERVER STARTED")
    print(f" Service UUID    : {SERVICE_UUID}")
    print(f" Order ID        : {order.order_id}")
    # Token provenance, on the banner rather than buried in the pull step. The
    # failure this prevents is silent: a handshake against a token from an
    # earlier flight looks identical to a wrong app, and the only symptom is
    # "AUTHENTICATION FAILED" with two hashes that mean nothing on their own.
    if order_file is not None:
        token_age = (time.time() - order_file.stat().st_mtime) / 60.0
        print(f" Token           : per-order, pulled {token_age:.0f} min ago")
    else:
        print(" Token           : ⚠️  HARDCODED BENCH TOKEN (--dev) - not a real delivery")
    print(f" Target GPS      : ({order.target_lat}, {order.target_lng})")
    print(f" Micro Geofence  : {geofence_radius_m():g}m "
          f"(delivery.ble_geofence_m; {GEOFENCE_MICRO_RADIUS_M:g}m if the GCS is unreachable)")
    print(f" Temporal Check  : {TEMPORAL_READINGS_REQUIRED} consistent readings "
          f"within {TEMPORAL_WINDOW_S:.0f}s (anti-spoofing)")
    print(f" GCS             : {GCS_BASE_URL}  (state cached every "
          f"{GCS_STATE_POLL_S:.1f}s, stale after {GCS_STATE_MAX_AGE_S:.1f}s)")
    print(f" Follow-me       : REMOVED — the drone never flies to the phone; a "
          f">{geofence_radius_m():.0f}m gap only closes if the recipient walks in")
    print(" Security Layers : HMAC Mutual Auth + Temporal Consistency + Micro Geofence "
          "+ Signed Payloads + RESULT read-back")
    print(" Status          : Advertising... Waiting for Mobile App connection.")
    print("=" * 60 + "\n")

    # Held in a local: asyncio keeps only weak refs to tasks, so an unreferenced
    # watcher could be garbage-collected mid-flight.
    watch = None
    if Path(args.cred).exists():
        watch = asyncio.create_task(_order_watch(service, args.order_file, args.cred))
    else:
        print(f"  ⚠️  [ORDER] no Firebase credentials at {args.cred} - order hot-reload "
              "OFF; a newly dispatched order needs: sudo systemctl restart aerix-ble")

    await asyncio.Event().wait()


if __name__ == "__main__":
    asyncio.run(main())
