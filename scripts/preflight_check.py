#!/usr/bin/env python3
"""Go / no-go before any flight: what is still in BENCH state?

    scripts/preflight_check.py                      # config only, runs anywhere
    scripts/preflight_check.py --fc                 # + the Pixhawk's own params
    scripts/preflight_check.py --config config/sim.yaml

Read-only. It changes nothing on the Pi or the FC; it reports. --fc needs
/dev/ttyACM0, so stop aerix-gcs first (CLAUDE.md §4).

Failsafes are deliberately OFF in config/real.yaml for bench work. That is
the operator's call, and this script does not override it. It makes the
state impossible to miss on the day of a flight, which is the evidence
review's item 7: "a live demo with these off invites the question you
least want".

Exit 0 = GO, 1 = NO-GO (at least one blocker), 2 = could not check.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from drone_stack.utils.config import Config  # noqa: E402

BLOCK, WARN, OK = "NO-GO", "WARN", "ok"
STICK_MODES = {"POSHOLD", "LOITER", "ALT_HOLD", "ALTHOLD", "STABILIZE"}
# ArduCopter mode numbers that hand altitude/throttle to the pilot's stick.
STABILIZE, ACRO = 0, 1


def check_config(c) -> list[tuple[str, str, str]]:
    """(level, item, detail) for everything the config can tell us."""
    out = []
    s, d, n = c["safety"], c["delivery"], c["navigation"]

    if not s.get("failsafes_enabled", True):
        out.append((BLOCK, "safety.failsafes_enabled",
                    "false - GCS-side battery / link-loss / geofence failsafes are OFF "
                    "(bench setting). Set true in config/real.yaml before flight."))
    else:
        out.append((OK, "safety.failsafes_enabled", "true"))

    ceiling = float(s["max_altitude_m"])
    for key, val in (("navigation.takeoff_altitude_m", n.get("takeoff_altitude_m")),
                     ("navigation.cruise_altitude_m", n.get("cruise_altitude_m")),
                     ("delivery.hover_altitude_m", d.get("hover_altitude_m")),
                     ("delivery.scan_altitude_m", d.get("scan_altitude_m")),
                     ("delivery.drop_climb_alt_m", d.get("drop_climb_alt_m"))):
        if val is not None and float(val) > ceiling:
            out.append((WARN, key, f"{val} m is above the {ceiling} m ceiling - it will be "
                                   "silently clamped, not flown"))

    mode = str(d.get("hover_mode", "GUIDED")).upper()
    out.append((BLOCK if mode in STICK_MODES else OK, "delivery.hover_mode",
                f"{mode}" + (" - takes altitude from the throttle stick; this broke the "
                             "airframe twice (CLAUDE.md §11)" if mode in STICK_MODES else "")))

    if d.get("auto_accept"):
        out.append((BLOCK, "delivery.auto_accept",
                    "true - an order would launch the aircraft with no human ACCEPT"))
    if float(s.get("geofence_radius_m", 0)) <= float(d.get("max_delivery_radius_m", 0)):
        out.append((WARN, "safety.geofence_radius_m",
                    f"{s.get('geofence_radius_m')} m <= max_delivery_radius_m "
                    f"{d.get('max_delivery_radius_m')} m - a legal delivery would trip it"))

    for blk in ("payload", "aux2_servo"):
        b = c[blk] if blk in c else {}
        if b and int(b.get("min_us", 1000)) <= 600 and int(b.get("max_us", 2000)) >= 2400:
            out.append((WARN, f"{blk}.min_us/max_us",
                        f"{b['min_us']}-{b['max_us']} us: full commissioning travel. Full "
                        "travel broke the mechanism on 2026-08-10; narrow it around the "
                        "measured lock/release points (CLAUDE.md §6)."))

    lv = float(s.get("battery_low_voltage", 0))
    if 0 < lv < 18.0:
        out.append((WARN, "safety.battery_low_voltage",
                    f"{lv} V looks like a 4S threshold; this airframe flies 6S"))
    return out


def servo_channels(c) -> list[int]:
    """FC outputs the servos are on - from config, never hardcoded: the
    payload has moved AUX1 -> AUX5 -> AUX4 (SERVO12) already."""
    return sorted({int(c[b]["out_channel"]) for b in ("payload", "aux2_servo")
                   if b in c and "out_channel" in c[b]})


def check_fc(params: dict, channels: list[int]) -> list[tuple[str, str, str]]:
    """(level, item, detail) from the autopilot's own parameters."""
    out = []

    def p(name):
        return params.get(name)

    def need(name, ok, level, bad_msg, good_msg=""):
        v = p(name)
        if v is None:
            out.append((WARN, name, "not readable"))
        elif ok(v):
            out.append((OK, name, f"{v:g} {good_msg}".strip()))
        else:
            out.append((level, name, f"{v:g} - {bad_msg}"))

    need("ARMING_CHECK", lambda v: int(v) == 1, BLOCK,
         "pre-arm checks are not all enabled (CLAUDE.md §7)")
    need("FS_THR_ENABLE", lambda v: int(v) != 0, BLOCK, "RC-loss failsafe disabled")
    need("FS_GCS_ENABLE", lambda v: int(v) != 0, BLOCK,
         "GCS-loss failsafe disabled - a Pi/link death mid-flight is unrecoverable")
    need("BATT_FS_LOW_ACT", lambda v: int(v) != 0, BLOCK, "no action on low battery")
    need("BATT_FS_CRT_ACT", lambda v: int(v) != 0, BLOCK, "no action on critical battery")
    need("FENCE_ENABLE", lambda v: int(v) != 0, WARN,
         "no FC-side geofence; only the Pi's (gone if the Pi dies)")
    need("PILOT_SPEED_DN", lambda v: 0 < v <= 150, WARN,
         "0 means PILOT_SPEED_UP (250 cm/s) - the 27 Aug hard-landing rate")
    stick = [f"FLTMODE{i}" for i in range(1, 7)
             if p(f"FLTMODE{i}") is not None and int(p(f"FLTMODE{i}")) in (STABILIZE, ACRO)]
    if stick:
        out.append((WARN, "FLTMODE1-6",
                    f"{', '.join(stick)} = STABILIZE/ACRO: throttle stick drives the motors "
                    "directly - a grab mid-delivery with the stick low is a free fall "
                    "(CLAUDE.md §11, outstanding)"))
    for ch in channels:
        lo, hi = p(f"SERVO{ch}_MIN"), p(f"SERVO{ch}_MAX")
        if lo is not None and hi is not None and lo <= 600 and hi >= 2400:
            out.append((WARN, f"SERVO{ch}_MIN/MAX", f"{lo:g}-{hi:g} us: full travel on the FC"))
    rtl = p("RTL_ALT")
    if rtl is not None and rtl > 600:
        out.append((WARN, "RTL_ALT", f"{rtl / 100:g} m: RTL climbs to this, above the ceiling"))
    return out


FC_PARAMS = ["ARMING_CHECK", "FS_THR_ENABLE", "FS_GCS_ENABLE", "BATT_FS_LOW_ACT",
             "BATT_FS_CRT_ACT", "FENCE_ENABLE", "PILOT_SPEED_DN", "RTL_ALT"] + \
            [f"FLTMODE{i}" for i in range(1, 7)]


def read_fc(port: str, channels: list[int]) -> dict:
    sys.path.insert(0, str(ROOT / "scripts"))
    from pymavlink import mavutil
    from read_safety_params import get_param
    m = mavutil.mavlink_connection(port, baud=115200)
    if not m.wait_heartbeat(timeout=15):
        raise RuntimeError("no heartbeat - is aerix-gcs still holding the port?")
    names = FC_PARAMS + [f"SERVO{ch}_{e}" for ch in channels for e in ("MIN", "MAX")]
    return {name: get_param(m, name) for name in names}


def verdict(rows) -> int:
    width = max(len(r[1]) for r in rows)
    for level, item, detail in rows:
        mark = {"NO-GO": "✗ NO-GO", "WARN": "! warn ", "ok": "✓ ok   "}[level]
        print(f"  {mark}  {item:<{width}}  {detail}")
    blockers = [r for r in rows if r[0] == BLOCK]
    warns = [r for r in rows if r[0] == WARN]
    print()
    if blockers:
        print(f"NO-GO: {len(blockers)} blocker(s), {len(warns)} warning(s). "
              "Bench work only.")
        return 1
    print(f"GO: no blockers, {len(warns)} warning(s) to read before take-off.")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=str(ROOT / "config" / "real.yaml"))
    ap.add_argument("--fc", action="store_true", help="also read the Pixhawk's params")
    ap.add_argument("--port", default="/dev/ttyACM0")
    a = ap.parse_args(argv)
    print(f"Pre-flight check — {a.config}\n")
    cfg = Config.load(a.config)
    rows = check_config(cfg)
    if a.fc:
        chans = servo_channels(cfg)
        try:
            rows += check_fc(read_fc(a.port, chans), chans)
        except Exception as exc:  # noqa: BLE001
            print(f"could not read the FC: {exc}")
            return 2
    else:
        rows.append((WARN, "FC parameters", "not checked - run with --fc on the Pi "
                                            "(aerix-gcs stopped)"))
    return verdict(rows)


if __name__ == "__main__":
    sys.exit(main())
