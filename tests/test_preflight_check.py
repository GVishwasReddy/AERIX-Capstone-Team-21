"""scripts/preflight_check.py turns every known bench setting into a NO-GO."""
from __future__ import annotations

import copy
import importlib.util
from pathlib import Path

from drone_stack.utils.config import Config

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("preflight_check",
                                               ROOT / "scripts" / "preflight_check.py")
pf = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pf)

FLIGHT_READY_FC = {"ARMING_CHECK": 1, "FS_THR_ENABLE": 1, "FS_GCS_ENABLE": 1,
                   "BATT_FS_LOW_ACT": 2, "BATT_FS_CRT_ACT": 1, "FENCE_ENABLE": 1,
                   "PILOT_SPEED_DN": 100, "RTL_ALT": 500, "SERVO12_MIN": 1000,
                   "SERVO12_MAX": 1600, "SERVO14_MIN": 1000, "SERVO14_MAX": 2000,
                   **{f"FLTMODE{i}": 5 for i in range(1, 7)}}


def levels(rows):
    return {item: level for level, item, _ in rows}


def real():
    return copy.deepcopy(Config.load(str(ROOT / "config/real.yaml")))


def test_failsafes_off_is_a_no_go():
    c = real()
    c["safety"]["failsafes_enabled"] = False
    assert levels(pf.check_config(c))["safety.failsafes_enabled"] == pf.BLOCK


def test_a_stick_driven_hover_mode_is_a_no_go():
    c = real()
    c["delivery"]["hover_mode"] = "POSHOLD"
    assert levels(pf.check_config(c))["delivery.hover_mode"] == pf.BLOCK


def test_auto_accept_in_flight_config_is_a_no_go():
    c = real()
    c["delivery"]["auto_accept"] = True
    assert levels(pf.check_config(c))["delivery.auto_accept"] == pf.BLOCK


def test_an_altitude_above_the_ceiling_is_named():
    c = real()
    c["delivery"]["scan_altitude_m"] = c["safety"]["max_altitude_m"] + 1
    assert levels(pf.check_config(c))["delivery.scan_altitude_m"] == pf.WARN


def test_a_flight_ready_fc_has_no_blockers():
    assert pf.BLOCK not in levels(pf.check_fc(FLIGHT_READY_FC, [12, 14])).values()


def test_each_disabled_fc_failsafe_is_a_no_go():
    for name in ("ARMING_CHECK", "FS_THR_ENABLE", "FS_GCS_ENABLE",
                 "BATT_FS_LOW_ACT", "BATT_FS_CRT_ACT"):
        fc = dict(FLIGHT_READY_FC, **{name: 0})
        assert levels(pf.check_fc(fc, [12, 14]))[name] == pf.BLOCK, name


def test_stabilize_on_the_mode_switch_is_flagged():
    fc = dict(FLIGHT_READY_FC, FLTMODE1=0, FLTMODE3=0)
    rows = pf.check_fc(fc, [12, 14])
    assert levels(rows)["FLTMODE1-6"] == pf.WARN
    assert "FLTMODE1, FLTMODE3" in next(d for _, i, d in rows if i == "FLTMODE1-6")


def test_the_exit_code_is_no_go_with_any_blocker(capsys):
    assert pf.verdict([(pf.OK, "a", ""), (pf.BLOCK, "b", "")]) == 1
    assert pf.verdict([(pf.OK, "a", ""), (pf.WARN, "b", "")]) == 0


def test_servo_travel_is_checked_on_the_configured_outputs():
    # The payload moved AUX5 -> AUX4 (SERVO12) on 2026-09-11; a hardcoded
    # SERVO13 check would have audited a pin nothing is wired to.
    assert pf.servo_channels(real()) == [12, 14]
    rows = pf.check_fc(dict(FLIGHT_READY_FC, SERVO12_MIN=500, SERVO12_MAX=2500), [12, 14])
    assert levels(rows)["SERVO12_MIN/MAX"] == pf.WARN
