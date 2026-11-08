"""Precision drop on the downward TF02-Pro (2026-09-26).

Requirement: after the verification and the BLE handshake, and BEFORE the
parcel is released, move ~1-2 m away from the recipient, descend to 1 m above
the ground (measured by the TF02-Pro), release, report the delivery complete,
climb back to 5 m and RTL - with the RPLIDAR C1 obstacle avoidance working the
whole time.

Geometry and timings are pinned here rather than read from the shipped
profile, so re-tuning real.yaml cannot silently change what these assert.
Time is advanced by back-dating the drop's own timestamps, never by sleeping.
"""
from __future__ import annotations

import copy
import math
import time

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import (
    DeliveryBleResult,
    FusedState,
    MissionPhase,
    Obstacle,
    ObstacleArray,
    PersonLockState,
    RangeReading,
    Waypoint,
)
from drone_stack.nodes.navigation_node import NavigationNode
from drone_stack.srv import ServiceRegistry, ServiceRequest
from drone_stack.utils.config import Config

_SCAN = {
    "person_scan_enabled": True, "scan_altitude_m": 4.0,
    "person_search_s": 25.0, "handshake_window_s": 25.0,
    "scan_orbit_radius_m": 2.0, "scan_orbit_points": 8, "scan_orbit_step_s": 3.0,
    "scan_descend_timeout_s": 8.0, "scan_max_total_s": 90.0,
    "locator_enabled": False, "locator_log_dir": "",
    "scan_pause_on_person": True, "scan_pause_conf": 0.30, "scan_pause_max_s": 6.0,
    "scan_resume_s": 1.5, "scan_found_grace_s": 3.0,
    "hover_altitude_m": 5.0,
}
_DROP = {
    "precision_drop_enabled": True, "drop_offset_m": 1.8, "drop_offset_tol_m": 0.3,
    "drop_min_person_m": 1.0, "drop_target_agl_m": 1.0, "drop_agl_tol_m": 0.15,
    "drop_abort_agl_m": 0.6, "drop_descent_step_m": 0.5,
    "drop_command_interval_s": 0.5, "drop_settle_s": 1.0,
    "drop_release_hold_s": 2.0, "drop_climb_alt_m": 5.0,
    "drop_climb_timeout_s": 15.0, "drop_approach_timeout_s": 10.0,
    "drop_timeout_s": 45.0, "drop_rangefinder_stale_s": 0.5,
    # 0 here so the flow tests start on one frame; TestTf02Continuity pins 1.0.
    "drop_rf_ready_s": 0.0, "drop_rf_gap_max_s": 1.5, "drop_rf_max_gaps": 3,
    "drop_under_jump_m": 0.7, "drop_over_jump_m": 1.5,
    "drop_avoid_abort_s": 5.0, "drop_max_attempts": 2,
    "drop_person_fresh_s": 3.0, "follow_me_max_m": 15.0,
}
DROP_WP = Waypoint(seq=1, x_m=0.0, y_m=20.0, alt_m=5.0, hold_s=20.0,
                   radius_m=1.5, kind="hover")
LEG = Waypoint(seq=0, x_m=0.0, y_m=10.0, alt_m=5.0, kind="nav")
ORDER = "ORDER_42"
PERSON = (0.0, 20.0)


def _nav(**over):
    raw = copy.deepcopy(Config.load().raw)
    raw.setdefault("delivery", {}).update({**_SCAN, **_DROP, **over})
    raw.setdefault("navigation", {})["avoidance_rtl_router"] = "pi"
    raw.setdefault("safety", {})["max_altitude_m"] = 5.0
    raw.setdefault("payload", {}).update(
        {"out_channel": 12, "lock_us": 1100, "release_us": 1410,
         "min_us": 500, "max_us": 2500})
    bus = MessageBus()
    sent: list = []
    results: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    bus.subscribe(Topics.DELIVERY_BLE_RESULT, results.append)
    services = ServiceRegistry()
    node = NavigationNode(bus, Config(raw), services)
    node._armed = True
    node._has_flown = True
    node._home = (12.9, 77.6)
    node._mode = "GUIDED"
    node._phase = MissionPhase.NAVIGATE
    node._mission.waypoints = [copy.copy(LEG), copy.copy(DROP_WP)]
    node._current_wp = 1
    node._avoid_min_alt = 0.0                     # takeoff gate is not under test
    _at(node, 20.0, alt=5.0)
    sent.clear()
    return node, sent, results, services


def _at(node, y, alt, x=0.0, yaw=0.0):
    node._fused = FusedState(x=x, y=y, alt_rel_m=alt, yaw=yaw, valid=True)


def _tf02(node, agl, valid=True, age=0.0):
    node._on_range(RangeReading(distance_m=agl, strength=900 if valid else 40,
                                valid=valid, stamp=time.time() - age))


def _commands(sent, name):
    return [c for c in sent if c.command == name]


def _locked(**over):
    """Holding over the drop point at 4 m, a person locked, window open."""
    node, sent, results, services = _nav(**over)
    node._enter_hover(node._mission.waypoints[1])
    _at(node, 20.0, alt=4.0)
    node._do_scan()                                   # descend -> search
    node._on_person_lock(PersonLockState(state="lock", score=0.8, lock_id=1,
                                         nx=0.0, ny=0.0, people=1,
                                         people_xy=[[0.0, 0.0, 0.8]]))
    node._do_scan()
    assert node._scan_stage == "locked"
    assert node._handshake_open()
    _tf02(node, 4.0)
    sent.clear()
    return node, sent, results, services


def _request(node, order=ORDER):
    return node._svc_precision_drop(ServiceRequest(name="precision_drop",
                                                   data={"order_id": order}))


def _start(person=PERSON, drone=(0.5, 20.0), **over):
    node, sent, results, services = _locked(**over)
    _at(node, drone[1], alt=4.0, x=drone[0])
    node._locked_person_enu = lambda: person
    r = _request(node)
    assert r.success, r.message
    return node, sent, results, services


def _tick(node, agl=None, alt=None):
    f = node._fused
    if alt is not None:
        _at(node, f.y, alt=alt, x=f.x)
    if agl is not None:
        _tf02(node, agl)
    node._do_hover()


def _age(node, s):
    """Pretend ``s`` seconds have passed for the drop's own clocks."""
    d = node._drop
    d.started -= s
    d.since -= s
    d.last_cmd -= s


def _arrive(node):
    """Fly the approach: put the aircraft on the drop target."""
    t = node._drop.target
    _at(node, t[1], alt=node._fused.alt_rel_m, x=t[0])
    _tick(node, agl=node._fused.alt_rel_m)
    assert node._drop.stage == "descend"


def _descend_to(node, agl):
    _at(node, node._fused.y, alt=agl, x=node._fused.x)
    _age(node, 1.0)
    _tick(node, agl=agl)


# --------------------------------------------------------------------------- #
# the request
# --------------------------------------------------------------------------- #
class TestRequest:
    def test_disabled_answers_legacy(self):
        node, *_ = _locked(precision_drop_enabled=False)
        r = _request(node)
        assert not r.success and r.data.get("mode") == "legacy"
        assert node._drop is None

    def test_disarmed_answers_legacy_so_the_bench_is_unchanged(self):
        node, *_ = _locked()
        node._armed = False
        r = _request(node)
        assert not r.success and r.data.get("mode") == "legacy"
        assert r.data.get("on_ground") is True, \
            "the peripheral needs this to refuse a real order's ground release"

    def test_airborne_legacy_is_not_flagged_on_ground(self):
        node, *_ = _locked(precision_drop_enabled=False)
        r = _request(node)
        assert r.data.get("mode") == "legacy" and not r.data.get("on_ground")

    def test_refused_before_a_lock(self):
        node, sent, _, _ = _nav()
        node._enter_hover(node._mission.waypoints[1])
        _at(node, 20.0, alt=4.0)
        node._do_scan()
        assert node._scan_stage == "search"
        _tf02(node, 4.0)
        r = _request(node)
        assert not r.success and "handshake" in r.message
        assert r.data.get("mode") != "legacy"

    def test_refused_without_a_tf02_reading(self):
        node, *_ = _locked()
        node._range = None
        r = _request(node)
        assert not r.success and "blind" in r.message

    def test_refused_on_a_no_target_or_stale_reading(self):
        node, *_ = _locked()
        _tf02(node, 45.0, valid=False)
        assert not _request(node).success
        _tf02(node, 4.0, age=2.0)
        assert not _request(node).success

    def test_refused_under_pilot_control(self):
        node, *_ = _locked()
        node._pilot_override = True
        assert not _request(node).success

    def test_accepted_closes_the_window_and_moves_away_from_the_person(self):
        node, sent, *_ = _start(person=(0.0, 20.0), drone=(0.5, 20.0))
        d = node._drop
        assert d.stage == "approach"
        assert not node._handshake_open(), "a second release window while descending"
        assert math.isclose(d.target[0], 1.8, abs_tol=1e-6)
        assert math.isclose(d.target[1], 20.0, abs_tol=1e-6)
        gotos = _commands(sent, "goto")
        assert gotos and math.isclose(gotos[-1].params["alt"], 4.0, abs_tol=1e-6)

    def test_without_the_person_scan_the_window_still_closes_mid_drop(self):
        # Scan disabled: _handshake_open used to be unconditionally True, so
        # only the drop's own gate stops a second release while descending.
        node, sent, _, _ = _nav(person_scan_enabled=False)
        node._enter_hover(node._mission.waypoints[1])
        _at(node, 20.0, alt=5.0)
        _tf02(node, 5.0)
        assert node._handshake_open()
        r = _request(node)
        assert r.success, r.message
        assert node._drop.target == (0.0, 20.0), "no lock: descends where it holds"
        assert not node._handshake_open()

    def test_same_order_twice_is_idempotent(self):
        node, sent, *_ = _start()
        n = node._drop_attempts[ORDER]
        r = _request(node)
        assert r.success and r.data["stage"] == "approach"
        assert node._drop_attempts[ORDER] == n

    def test_directly_overhead_steps_sideways_not_backwards(self):
        node, *_ = _start(person=(0.0, 20.0), drone=(0.0, 20.0))
        t = node._drop.target
        # yaw 0 = north; sideways is east/west, never south (behind).
        assert math.isclose(abs(t[0]), 1.8, abs_tol=1e-6)
        assert math.isclose(t[1], 20.0, abs_tol=1e-6)

    def test_already_at_the_offset_descends_in_place(self):
        node, sent, *_ = _start(person=(0.0, 20.0), drone=(1.7, 20.0))
        assert node._drop.target == (1.7, 20.0)


# --------------------------------------------------------------------------- #
# the flight
# --------------------------------------------------------------------------- #
class TestSequence:
    def test_full_drop_release_report_climb_rtl(self):
        node, sent, results, _ = _start()
        _arrive(node)
        assert math.isclose(node._drop.ground_ref, 0.0, abs_tol=1e-6)

        # Stepped descent: never more than drop_descent_step_m below where it is.
        sent.clear()
        _descend_to(node, 4.0)
        goto = _commands(sent, "goto")[-1]
        assert math.isclose(goto.params["alt"], 3.5, abs_tol=1e-6)
        _descend_to(node, 1.3)
        goto = _commands(sent, "goto")[-1]
        assert math.isclose(goto.params["alt"], 1.0, abs_tol=1e-6), \
            "last step must stop AT the release height, not below it"
        assert not _commands(sent, "set_servo")

        _descend_to(node, 1.05)
        assert node._drop.stage == "settle"
        _tick(node, agl=1.05)
        assert not _commands(sent, "set_servo"), "released before settling"

        _age(node, 1.1)
        _tick(node, agl=1.02)
        assert node._drop.stage == "release"
        servo = _commands(sent, "set_servo")
        assert [(c.params["channel"], c.params["pwm"]) for c in servo] == [(12, 1410)]
        assert [(r.order_id, r.success) for r in results] == [(ORDER, True)], \
            "delivery complete must be reported at the release"
        assert node._ble_delivered_at is None, "own report started the early-RTL clock"

        _age(node, 2.1)
        _tick(node, agl=1.0)
        assert node._drop.stage == "climb"
        assert _commands(sent, "set_servo")[-1].params["pwm"] == 1100
        assert math.isclose(_commands(sent, "goto")[-1].params["alt"], 5.0, abs_tol=1e-6)

        _tick(node, agl=4.8, alt=4.8)
        assert node._drop.stage == "done"
        assert node._phase == MissionPhase.RTL
        fields = node._scan_state_fields()
        assert fields["drop_stage"] == "done" and fields["drop_order_id"] == ORDER

    def test_mission_state_carries_the_stage(self):
        node, *_ = _start()
        f = node._scan_state_fields()
        assert f["drop_stage"] == "approach" and f["drop_order_id"] == ORDER
        assert f["handshake_open"] is False


# --------------------------------------------------------------------------- #
# safety
# --------------------------------------------------------------------------- #
def _obstacle(dist, bearing):
    r = math.radians(bearing)
    return Obstacle(distance_m=dist, bearing_deg=bearing,
                    x_m=dist * math.cos(r), y_m=-dist * math.sin(r), num_points=10)


class TestSafety:
    def test_avoidance_wins_every_tick_and_the_drop_commands_nothing(self):
        node, sent, *_ = _start()
        _arrive(node)
        node._obstacles = ObstacleArray(obstacles=[_obstacle(0.6, 90.0)])
        sent.clear()
        _age(node, 1.0)
        _tick(node, agl=4.0)
        assert _commands(sent, "velocity"), "hover avoidance did not act mid-drop"
        assert not _commands(sent, "goto"), "the drop fought the avoidance"
        assert node._drop.stage == "descend"

    def test_avoidance_too_long_aborts_with_the_parcel(self):
        node, sent, results, _ = _start()
        _arrive(node)
        node._obstacles = ObstacleArray(obstacles=[_obstacle(0.6, 90.0)])
        _tick(node, agl=4.0)
        node._drop.avoid_since -= 6.0
        _tick(node, agl=4.0)
        assert node._drop.stage == "abort"
        assert not _commands(sent, "set_servo") and not results

    def test_obstacle_clear_holds_at_the_current_height(self):
        node, sent, *_ = _start()
        _arrive(node)
        _descend_to(node, 2.5)
        node._obstacles = ObstacleArray(obstacles=[_obstacle(0.6, 90.0)])
        _tick(node, agl=2.5)
        node._obstacles = ObstacleArray(obstacles=[])
        _at(node, 20.0, alt=2.5, x=1.0)
        sent.clear()
        _tick(node, agl=2.5)
        goto = _commands(sent, "goto")[0]
        assert math.isclose(goto.params["alt"], 2.5, abs_tol=1e-6), \
            "released avoidance snapped back up to the scan altitude"
        assert node._drop.target == (1.0, 20.0)

    def test_something_under_the_aircraft_aborts(self):
        node, sent, results, _ = _start()
        _arrive(node)
        _descend_to(node, 3.0)
        _at(node, node._fused.y, alt=2.5, x=node._fused.x)
        _tick(node, agl=1.5)                  # baro says 2.5 m, TF02 says 1.5 m
        assert node._drop.stage == "abort"
        assert "under" in node._drop.note
        assert not results

    def test_losing_the_tf02_holds_then_aborts_before_release(self):
        node, sent, results, _ = _start()
        _arrive(node)
        _descend_to(node, 2.0)
        _tf02(node, 2.0, age=1.0)
        node._do_hover()
        assert node._drop.stage == "descend", "a short gap must hold, not abort"
        node._drop.rf_lost_since -= 1.6       # past drop_rf_gap_max_s
        node._do_hover()
        assert node._drop.stage == "abort"
        assert "lost" in node._drop.note
        assert not _commands(sent, "set_servo") and not results

    def test_below_the_floor_aborts(self):
        node, *_ = _start()
        _arrive(node)
        _at(node, node._fused.y, alt=0.55, x=node._fused.x)
        _tick(node, agl=0.55)
        assert node._drop.stage == "abort"

    def test_overall_timeout_aborts(self):
        node, *_ = _start()
        _arrive(node)
        node._drop.started -= 46.0
        _tick(node, agl=4.0)
        assert node._drop.stage == "abort"

    def test_abort_climbs_back_and_reopens_the_window(self):
        node, sent, results, _ = _start()
        _arrive(node)
        node._drop.started -= 46.0
        sent.clear()
        _tick(node, agl=4.0, alt=3.0)
        assert node._drop.stage == "abort"
        assert math.isclose(_commands(sent, "goto")[-1].params["alt"], 4.0, abs_tol=1e-6)
        _tick(node, agl=3.9, alt=3.9)
        assert node._drop.stage == "aborted"
        assert node._scan_stage == "locked" and node._handshake_open()
        assert node._scan_state_fields()["drop_stage"] == "aborted"
        # the phone may retry
        _tf02(node, 3.9)
        assert _request(node).success

    def test_attempts_used_up_goes_home_with_the_parcel(self):
        node, sent, results, _ = _start(drop_max_attempts=1)
        node._drop_abort("test")
        _tick(node, agl=4.0, alt=4.0)
        assert node._drop.stage == "aborted"
        assert node._phase == MissionPhase.RTL
        assert not results

    def test_too_close_to_the_person_after_the_approach_aborts(self):
        node, *_ = _start()
        node._drop.person = (0.0, 20.0)
        _at(node, 20.0, alt=4.0, x=0.5)
        node._drop.since -= 11.0              # approach timed out short of the target
        _tick(node, agl=4.0)
        assert node._drop.stage == "abort"

    def test_pilot_override_mid_drop_is_recorded_not_flown(self):
        node, sent, results, _ = _start()
        _arrive(node)
        node._pilot_override = True
        sent.clear()
        node._drop_reconcile()
        assert node._drop.stage == "aborted"
        assert not sent and not results

    def test_a_legacy_release_mid_drop_cancels_it(self):
        node, sent, *_ = _start()
        node._on_ble_delivery_result(DeliveryBleResult(order_id=ORDER, success=True))
        assert node._drop.stage == "done"
        assert node._ble_delivered_at is not None   # pre-existing early-RTL path


class TestGotoGuard:
    def _goto(self, node, **data):
        return node._svc_goto_gps(ServiceRequest(name="goto_gps", data=data))

    def test_goto_refused_mid_drop(self):
        node, *_ = _start()
        assert not self._goto(node, lat=12.9001, lon=77.6001).success

    def test_invalid_coordinates_refused(self):
        node, *_ = _nav()
        for lat, lon in ((0.0, 0.0), (91.0, 1.0), (float("nan"), 1.0)):
            assert not self._goto(node, lat=lat, lon=lon).success

    def test_follow_me_never_arms_or_starts_a_flight(self):
        node, sent, *_ = _nav()
        node._armed = False
        node._phase = MissionPhase.IDLE
        r = self._goto(node, lat=12.9001, lon=77.6001, source="ble_follow_me")
        assert not r.success
        assert not _commands(sent, "arm")

    def test_follow_me_far_from_the_hold_refused(self):
        node, *_ = _nav(person_scan_enabled=False)
        node._enter_hover(node._mission.waypoints[1])
        # ~110 m north of home: ~90 m from the hold point
        r = self._goto(node, lat=12.901, lon=77.6, source="ble_follow_me")
        assert not r.success and "refused" in r.message

    def test_operator_goto_unchanged(self):
        node, sent, *_ = _nav()
        node._phase = MissionPhase.IDLE
        r = self._goto(node, lat=12.9001, lon=77.6001)
        assert r.success and node._phase == MissionPhase.MANUAL


# --------------------------------------------------------------------------- #
# TF02-Pro continuity (27 Sep: a flickering TF02 burned both attempts in 1.4 s)
# --------------------------------------------------------------------------- #
def _feed(node, ages, valid=None):
    """A clean history of frames at these ages (oldest first)."""
    node._range = None
    node._range_since = None
    for i, a in enumerate(ages):
        _tf02(node, 4.0, valid=True if valid is None else valid[i], age=a)


class TestTf02Continuity:
    def test_one_fresh_frame_is_not_steady_and_spends_no_attempt(self):
        node, *_ = _locked(drop_rf_ready_s=1.0)
        _feed(node, [0.0])
        r = _request(node)
        assert not r.success and "steady" in r.message
        assert node._drop is None
        assert node._drop_attempts.get(ORDER, 0) == 0, "a refusal spent an attempt"
        _feed(node, [1.2, 0.9, 0.6, 0.3, 0.0])
        assert _request(node).success

    def test_a_gap_in_the_run_restarts_the_clock(self):
        node, *_ = _locked(drop_rf_ready_s=1.0)
        _feed(node, [1.5, 1.2, 0.3, 0.0])     # 0.9 s hole > drop_rangefinder_stale_s
        assert not _request(node).success

    def test_an_invalid_frame_restarts_the_clock(self):
        node, *_ = _locked(drop_rf_ready_s=1.0)
        _feed(node, [1.5, 1.2, 0.9, 0.6, 0.3, 0.0],
              valid=[True, True, True, False, True, True])
        assert not _request(node).success

    def test_a_gap_in_the_approach_waits_instead_of_aborting(self):
        node, sent, results, _ = _start()
        t = node._drop.target
        _at(node, t[1], alt=4.0, x=t[0])
        _tf02(node, 4.0, age=1.0)
        node._do_hover()
        d = node._drop
        assert d.stage == "approach", "the approach aborted on a height it never uses"
        assert d.rf_gaps == 0
        _tick(node, agl=4.0)
        assert node._drop.stage == "descend"
        assert math.isclose(node._drop.ground_ref, 0.0, abs_tol=1e-6)

    def test_a_gap_mid_descent_freezes_the_altitude(self):
        node, sent, *_ = _start()
        _arrive(node)
        _descend_to(node, 2.5)
        sent.clear()
        _tf02(node, 2.5, age=1.0)
        node._do_hover()
        gotos = _commands(sent, "goto")
        assert len(gotos) == 1 and math.isclose(gotos[0].params["alt"], 2.5, abs_tol=1e-6), \
            "the last descent step kept pulling the aircraft down, blind"
        _age(node, 1.0)
        node._do_hover()
        assert len(_commands(sent, "goto")) == 1, "descended again during the gap"
        assert node._drop.stage == "descend"
        _age(node, 1.0)
        _tick(node, agl=2.5)
        assert math.isclose(_commands(sent, "goto")[-1].params["alt"], 2.0, abs_tol=1e-6)
        assert node._drop.rf_lost_since is None

    def test_repeated_gaps_abort_as_unreliable(self):
        node, sent, results, _ = _start()
        _arrive(node)
        _descend_to(node, 3.0)
        for _ in range(4):
            _tf02(node, 3.0, age=1.0)
            node._do_hover()
            if node._drop.stage == "abort":
                break
            _tick(node, agl=3.0)
        assert node._drop.stage == "abort"
        assert "unreliable" in node._drop.note and node._drop.rf_gaps == 4
        assert not results

    def test_a_gap_below_the_floor_by_the_barometer_aborts_at_once(self):
        node, *_ = _start()
        _arrive(node)
        _at(node, node._fused.y, alt=0.5, x=node._fused.x)
        _tf02(node, 0.5, age=1.0)
        node._do_hover()
        assert node._drop.stage == "abort"
        assert "barometer" in node._drop.note

    def test_the_settle_starts_over_after_a_gap(self):
        node, sent, results, _ = _start()
        _arrive(node)
        _descend_to(node, 1.3)
        _descend_to(node, 1.05)
        assert node._drop.stage == "settle"
        _age(node, 1.1)                       # settle time would be up...
        _tf02(node, 1.05, age=1.0)
        node._do_hover()                      # ...but the height went blind
        _tick(node, agl=1.02)
        assert node._drop.stage == "settle", "released on one tick of a returned reading"
        assert not _commands(sent, "set_servo") and not results
        _age(node, 1.1)
        _tick(node, agl=1.02)
        assert node._drop.stage == "release"
