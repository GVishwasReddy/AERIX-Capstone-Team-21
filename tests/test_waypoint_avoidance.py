"""Avoidance at the waypoint, over the drop point, and across the full 250 deg.

Flight 2026-10-04 17:18 (flight_20261004_171836.jsonl):
  * SLOW latched on a wall BEYOND the drop point; the steer bent the course
    into an orbit of the waypoint, it never arrived, the person scan never ran.
  * The yaw gate "held" by sending nothing, so the last steer velocity (1 m/s)
    kept flying at an obstacle for the FC's guided timeout.
  * STOP only looked +/-50 deg off the NOSE while the aircraft strafed sideways.
And the scan orbit only judged the END of a leg, never the chord it flew.
"""
from __future__ import annotations

import math

from drone_stack.msg import FusedState, MissionPhase, ObstacleArray
from drone_stack.nodes.navigation_node import CLEAR, SLOW, STOP

import test_person_scan as tps
from test_person_scan import _at, _commands, _obstacle, _seen
from drone_stack.utils.config import Config

#: The aircraft's avoidance geometry (config/real.yaml, 2026-10-04), pinned so
#: the default profile's 2.5 m bench values cannot hide what flew.
_REAL_NAV = {
    "avoidance_distance_m": 11.5, "avoidance_stop_m": 1.5,
    "avoidance_vfh_safety_radius_m": 2.5, "avoidance_front_half_deg": 50.0,
    "avoidance_side_half_deg": 125.0, "waypoint_radius_m": 1.0,
    "avoidance_travel_cone_min_ms": 0.3,
}


def _real_load(_orig=Config.load):
    def load(*a, **k):
        cfg = _orig(*a, **k)
        cfg._data.setdefault("navigation", {}).update(_REAL_NAV)
        return cfg
    return load


def _nav(**scan):
    Config.load, orig = _real_load(), Config.load
    try:
        return tps._nav(**scan)
    finally:
        Config.load = orig


def _searching(**scan):
    Config.load, orig = _real_load(), Config.load
    try:
        return tps._searching(**scan)
    finally:
        Config.load = orig


def _moving(node, y, east_ms=0.0, north_ms=0.0, x=0.0, yaw=0.0, alt=4.0):
    # FusedState is ENU: vx east, vy north.
    node._fused = FusedState(x=x, y=y, alt_rel_m=alt, yaw=yaw,
                             vx=east_ms, vy=north_ms, valid=True)


def _field(*obs):
    return ObstacleArray(obstacles=list(obs))


# -- 250 deg: brake along the direction of travel ---------------------------
class TestTravelCone:
    def test_the_nose_cone_alone_misses_a_strafe_into_something(self):
        node, _, _ = _nav()
        av = node._avoider
        side = _field(_obstacle(0.8, 90.0))
        assert av.evaluate(side)[0] == CLEAR          # old behaviour: nose-only
        assert av.evaluate(side, travel_deg=90.0)[0] == STOP

    def test_the_masked_rear_is_never_judged(self):
        node, _, _ = _nav()
        av = node._avoider
        rear = _field(_obstacle(0.8, 160.0))
        assert av.evaluate(rear, travel_deg=160.0)[0] == CLEAR

    def test_travel_bearing_from_the_fused_velocity(self):
        node, _, _ = _nav()
        _moving(node, 15.0, east_ms=1.0)              # nose north, moving east
        assert abs(node._travel_bearing_deg() - 90.0) < 1.0
        _moving(node, 15.0, east_ms=-1.0)
        assert abs(node._travel_bearing_deg() + 90.0) < 1.0
        _moving(node, 15.0, east_ms=0.1)              # hover drift: no direction
        assert node._travel_bearing_deg() is None

    def test_navigate_brakes_on_an_obstacle_in_the_strafe_path(self):
        node, sent, _ = _nav()
        _moving(node, 12.0, east_ms=1.0)
        node._obstacles = _field(_obstacle(0.8, 90.0))
        node._do_navigate()
        assert _commands(sent, "brake"), "strafed into an obstacle 90 deg off the nose"


# -- at the waypoint ----------------------------------------------------------
class TestArrivalWithObstacles:
    def test_an_obstacle_beyond_the_waypoint_does_not_stop_the_arrival(self):
        node, sent, _ = _nav()
        _at(node, 17.0, alt=4.0)                       # 3 m short of the drop point
        node._obstacles = _field(_obstacle(8.0, 0.0))  # a wall 5 m past it
        assert node._avoid_decision()[0] == SLOW
        node._do_navigate()
        assert not _commands(sent, "velocity"), "steered round a wall past the waypoint"
        gotos = _commands(sent, "goto")
        assert gotos, "the waypoint goto was not flown"
        assert not node._avoiding

    def test_an_obstacle_short_of_the_waypoint_still_steers(self):
        node, sent, _ = _nav()
        _at(node, 12.0, alt=4.0)                       # 8 m out
        node._obstacles = _field(_obstacle(4.0, 0.0))  # in the way
        node._do_navigate()
        assert _commands(sent, "velocity"), "flew straight at an obstacle on the leg"

    def test_close_to_the_drop_point_with_an_obstacle_it_arrives_short(self):
        node, sent, _ = _nav()
        _at(node, 18.0, alt=4.0)                       # 2 m out, radius 1.5
        node._obstacles = _field(_obstacle(3.0, 0.0))  # 1 m past the drop point
        node._do_navigate()
        assert node._phase == MissionPhase.HOVER, "kept circling for a clean approach"
        assert node._scan_stage == "descend"
        # The descent leg onto the drop point passes 1 m from the obstacle:
        # hold where we stopped instead.
        assert node._scan_target == (0.0, 18.0)


# -- the scan: legs, not just endpoints --------------------------------------
class TestScanLegs:
    def test_a_chord_through_an_obstacle_is_blocked(self):
        node, _, _ = _searching()
        _at(node, 20.0, alt=3.0)
        node._obstacles = _field(_obstacle(3.5, 0.0))       # at (0, 23.5)
        assert not node._near_obstacle((0.0, 27.5))         # endpoint is clean...
        assert node._leg_blocked((0.0, 27.5))               # ...the leg is not
        assert not node._leg_blocked((0.0, 16.0))           # straight away from it

    def test_backing_off_something_already_close_is_allowed(self):
        node, _, _ = _searching()
        _at(node, 20.0, alt=3.0)
        node._obstacles = _field(_obstacle(2.0, 0.0))       # inside the hold distance
        assert not node._leg_blocked((-2.0, 20.0))
        assert not node._leg_blocked((0.0, 17.0))
        assert node._leg_blocked((0.0, 21.5))

    def test_a_pending_leg_that_became_blocked_is_not_flown(self):
        node, sent, _ = _searching()
        _at(node, 20.0, alt=3.0)
        node._scan_pending = (0.0, 22.0)                     # dead ahead: leg is ready
        node._scan_pending_since = 1e18
        node._obstacles = _field(_obstacle(3.0, 0.0))        # appeared during the turn
        node._scan_orbit_step(0.0)
        gotos = _commands(sent, "goto")
        assert node._scan_pending is None
        assert node._scan_target == (0.0, 20.0), "flew the orbit leg toward the obstacle"
        assert gotos

    def test_orbit_points_whose_chord_is_blocked_are_skipped(self):
        # Pushed 6 m south of the centre by the hover dodge; a post sits
        # between here and the circle. Several orbit points have clean ENDS
        # (e.g. (0, 22) is 4.5 m from it) but every leg there runs through it.
        node, sent, _ = _searching()
        _at(node, 14.0, alt=3.0, yaw=0.0)
        node._obstacles = _field(_obstacle(3.5, 0.0))       # at (0, 17.5)
        node._scan_target = (0.0, 14.0)      # the dodge re-anchored here (_hold_here)
        assert not node._near_obstacle((0.0, 22.0))
        node._scan_next_step = 0.0
        node._scan_orbit_step(1.0)
        assert node._scan_pending is None, f"picked a leg through the post: {node._scan_pending}"
        assert "holding" in node._status_message


# -- stop circling once a person is seen ------------------------------------
class TestStopCircling:
    def test_it_stays_on_a_person_in_view_with_no_lock(self):
        node, sent, _ = _searching(scan_pause_max_s=0.0)
        _seen(node)
        node._do_scan()
        assert node._scan_paused_at is not None
        node._scan_paused_at -= 60.0                         # long after the old 6 s
        node._scan_next_step = 0.0
        sent.clear()
        for _ in range(5):
            _seen(node)
            node._do_scan()
        assert node._scan_paused_at is not None, "resumed circling with the person in view"
        assert not _commands(sent, "goto")

    def test_the_shipped_default_is_to_stay(self):
        assert float(Config.load().section("delivery")["scan_pause_max_s"]) <= 0.0
