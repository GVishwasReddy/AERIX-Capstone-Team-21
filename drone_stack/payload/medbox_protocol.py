"""MedBox BLE wire protocol - the Pi-side mirror of medbox firmware/src/main.cpp.

Pure stdlib on purpose: imported by the bridge (its own bleak venv, no yaml),
by the GCS hub, and by the tests. Every table here is APPEND ONLY and must stay
in step with the firmware's enums and its pushStatus() keys.
"""
from __future__ import annotations

import hashlib
import hmac
import json
from pathlib import Path

SERVICE_UUID = "aad1880d-ae20-4b48-8f1b-79b6bb371248"
STATUS_UUID = "d3bb4bf2-2c47-4630-82fb-ffe15dd26718"   # read + notify, compact JSON
NONCE_UUID = "67fc87e8-19b3-47fb-90d7-207717604167"    # read, single use
CMD_UUID = "a47403e2-3eef-4efa-92fc-d8685a4dd86b"      # write "CMD|nonce|sig"

STATES = ["UNARMED", "ARMED", "COMPROMISED", "DELIVERED"]
REASONS = ["none", "lid opened", "temperature out of range", "humidity out of range",
           "impact/drop", "vigorous shaking", "sensor fault",
           "impact in an unconfirmed drop"]
BATT_LEVELS = ["n/a", "ok", "LOW", "CRITICAL"]
DROP_PHASES = ["none", "expecting", "landing"]

COMMANDS = ("ARM_DROP", "CONFIRM_DROP", "CANCEL_DROP", "RESET")

# Navigator drop_stage -> the command that keeps the box in step with it.
# ARM_DROP goes out BEFORE the servo fires: a 1 m fall takes ~0.45 s, quicker
# than one /api/state poll, so waiting for "release" would let the landing
# jolt condemn the box before anyone told it a drop was coming.
STAGE_COMMAND = {
    "approach": "ARM_DROP", "descend": "ARM_DROP", "settle": "ARM_DROP",
    "release": "CONFIRM_DROP", "climb": "CONFIRM_DROP", "done": "CONFIRM_DROP",
    "aborted": "CANCEL_DROP",
}


def usb_bt_adapter(base: str | Path = "/sys/class/bluetooth") -> str | None:
    """hciN of the first USB-attached Bluetooth controller, else None.

    By sysfs bus path, never by number: hciN follows registration order, so a
    re-plugged dongle or a late UART attach can swap hci0 and hci1. Lives here
    (stdlib) rather than in the bridge so the GCS test venv, which has no
    bleak, can test the rule that keeps the bridge off the handshake radio.
    """
    base = Path(base)
    if not base.is_dir():
        return None
    for dev in sorted(base.iterdir()):
        if not dev.name.startswith("hci") or ":" in dev.name:
            continue
        if "/usb" in str(dev.resolve()):
            return dev.name
    return None


def _name(table: list[str], i) -> str:
    try:
        return table[int(i)]
    except (TypeError, ValueError, IndexError):
        return f"?{i}"


def decode_status(raw: bytes | str) -> dict:
    """Compact STATUS JSON -> the readable dict the hub, UI and OLED use.

    Raises ValueError on anything that is not a v2 status object.
    """
    if isinstance(raw, (bytes, bytearray)):
        raw = bytes(raw).decode("utf-8", "replace")
    s = json.loads(raw)
    if not isinstance(s, dict) or s.get("v") != 2:
        raise ValueError(f"not a MedBox v2 status: {raw[:60]!r}")
    t, h, bv = s.get("t", -99), s.get("h", -1), s.get("bv", 0)
    ack = str(s.get("a", ""))
    ack_cmd, _, ack_res = ack.partition(":")
    faults = int(s.get("f", 0))
    return {
        "seq": int(s.get("q", 0)),
        "state": _name(STATES, s.get("s")),
        "reason": _name(REASONS, s.get("r")),
        "reason_value": s.get("rv", 0),
        "temp_c": None if t <= -99 else t,
        "humidity": None if h < 0 else h,
        "shake_rms_g": s.get("sh", 0),
        "peak_g": s.get("pk", 0),
        "max_shake_g": s.get("ms", 0),
        "max_shock_g": s.get("mk", 0),
        "lid_closed": bool(s.get("l", 0)),
        "armed_s": int(s.get("up", 0)),
        "batt_v": bv if bv > 0 else None,
        "batt_level": _name(BATT_LEVELS, s.get("bl")),
        "power_interruptions": int(s.get("pi", 0)),
        "drop_phase": _name(DROP_PHASES, s.get("d")),
        "landing_peak_g": s.get("lg", 0),
        "opened_at_dest": bool(s.get("o", 0)),
        "imu_fault": bool(faults & 1),
        "dht_fault": bool(faults & 2),
        "ack_cmd": ack_cmd,
        "ack_result": ack_res,
        "ack_count": int(s.get("an", 0)),
    }


def load_key(path: str | Path) -> bytes:
    """The shared HMAC key: 64 hex chars (same bytes as medbox_secret.h)."""
    text = Path(path).read_text().strip()
    key = bytes.fromhex(text)
    if len(key) < 16:
        raise ValueError(f"{path}: key too short ({len(key)} bytes)")
    return key


def sign(key: bytes, nonce: str, cmd: str) -> str:
    """HMAC-SHA256(key, nonce|cmd), first 16 bytes as 32 hex - firmware signFor()."""
    return hmac.new(key, f"{nonce}|{cmd}".encode(), hashlib.sha256).hexdigest()[:32]


def frame_command(key: bytes, nonce: str, cmd: str) -> bytes:
    if cmd not in COMMANDS:
        raise ValueError(f"unknown MedBox command {cmd!r}")
    return f"{cmd}|{nonce}|{sign(key, nonce, cmd)}".encode()
