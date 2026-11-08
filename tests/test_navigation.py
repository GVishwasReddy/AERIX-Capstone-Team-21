"""Unit tests for navigation: collision avoidance, services, state (Phase 6)."""
from __future__ import annotations

from drone_stack.bus import MessageBus
from drone_stack.msg import MissionPhase, Obstacle, ObstacleArray
from drone_stack.nodes.navigation_node import CLEAR, SLOW, STOP, CollisionAvoider, NavigationNode
from drone_stack.srv import ServiceRegistry
from drone_stack.utils.config import Config


def _avoider() -> CollisionAvoider:
    return CollisionAvoider(Config.load())


def _front(distance: float) -> ObstacleArray:
    return ObstacleArray(obstacles=[Obstacle(distance_m=distance, bearing_deg=0.0)])


def test_avoider_stop_slow_clear():
    av = _avoider()
    assert av.evaluate(_front(1.0))[0] == STOP
    assert av.evaluate(_front(2.0))[0] == SLOW
    assert av.evaluate(_front(6.0))[0] == CLEAR


def test_avoider_ignores_obstacles_behind():
    av = _avoider()
    behind = ObstacleArray(obstacles=[Obstacle(distance_m=1.0, bearing_deg=175.0)])
    assert av.evaluate(behind)[0] == CLEAR


def test_avoider_empty_is_clear():
    av = _avoider()
    assert av.evaluate(None)[0] == CLEAR
    assert av.evaluate(ObstacleArray())[0] == CLEAR


def _nav() -> tuple[NavigationNode, ServiceRegistry]:
    services = ServiceRegistry()
    node = NavigationNode(MessageBus(), Config.load(), services)
    return node, services


def test_services_registered():
    _, services = _nav()
    for name in ("load_mission", "start_mission", "rtl", "emergency_stop"):
        assert services.has(name)


def test_load_and_start_mission():
    _, services = _nav()
    resp = services.call(
        "load_mission",
        waypoints=[{"x_m": 5.0, "y_m": 0.0}, {"x_m": 10.0, "y_m": 5.0}],
    )
    assert resp.success
    started = services.call("start_mission")
    assert started.success
    status = services.call("mission_status")
    assert status.data["total_wp"] == 2


def test_start_without_mission_fails():
    # A freshly constructed node has no mission (the demo is loaded in on_start).
    services = ServiceRegistry()
    NavigationNode(MessageBus(), Config.load(), services)
    assert not services.call("start_mission").success


def test_emergency_stop_sets_phase():
    node, services = _nav()
    services.call("emergency_stop")
    assert node._phase == MissionPhase.EMERGENCY
    assert services.call("mission_status").data["phase"] == "EMERGENCY"


# ---------------------------------------------------------- flight altitude --
# GCS altitude slider (2026-09-28). One number drives takeoff, the cruise legs,
# the drop-point arrival and the climb-back; the person scan keeps its own
# height. Each phase has its own rule so a slider change can never fight a
# climb in progress, the scan/drop, or a pilot who has taken over.
def _nav_with_route():
    node, services = _nav()
    node._alt_ceiling = 10.0            # deterministic, independent of config
    assert services.call(
        "load_mission",
        waypoints=[{"x_m": 5.0, "y_m": 0.0}, {"x_m": 10.0, "y_m": 5.0},
                   {"x_m": 15.0, "y_m": 5.0}],
    ).success
    return node, services


def test_flight_altitude_service_registered():
    _, services = _nav()
    assert services.has("set_flight_altitude")


def test_flight_altitude_on_the_ground_is_stored_for_next_takeoff():
    node, services = _nav_with_route()
    scan_before = node._scan_alt
    r = services.call("set_flight_altitude", alt_m=7.0)
    assert r.success, r.message
    assert r.data["alt_m"] == 7.0 and r.data["applied_now"] is False
    assert (node._takeoff_alt, node._cruise_alt, node._hover_alt, node._drop_climb_alt) == (7.0,) * 4
    assert node._scan_alt == scan_before, "the person scan keeps its own height"


def test_flight_altitude_is_clamped_to_the_ceiling_and_says_so():
    node, services = _nav_with_route()
    node._alt_ceiling = 5.0
    r = services.call("set_flight_altitude", alt_m=9.0)
    assert r.success
    assert r.data["alt_m"] == 5.0 and r.data["requested_m"] == 9.0
    assert "ceiling" in r.message
    assert node._cruise_alt == 5.0


def test_flight_altitude_range_is_2_to_10():
    node, services = _nav_with_route()
    assert services.call("set_flight_altitude", alt_m=0.5).data["alt_m"] == 2.0
    assert services.call("set_flight_altitude", alt_m=50).data["alt_m"] == 10.0


def test_flight_altitude_rejects_garbage():
    _, services = _nav_with_route()
    for bad in (None, "high", float("nan"), float("inf")):
        assert not services.call("set_flight_altitude", alt_m=bad).success, bad


def test_flight_altitude_while_navigating_is_flown_now():
    node, services = _nav_with_route()
    node._phase = MissionPhase.NAVIGATE
    node._current_wp = 1
    node._last_goto_wp = 1
    before_passed = node._mission.waypoints[0].alt_m
    r = services.call("set_flight_altitude", alt_m=6.5)
    assert r.data["applied_now"] is True
    assert node._last_goto_wp == -1, "current goto must be re-issued at the new height"
    assert [w.alt_m for w in node._mission.waypoints[1:]] == [6.5, 6.5]
    assert node._mission.waypoints[0].alt_m == before_passed, "flown legs are history"


def test_flight_altitude_during_takeoff_keeps_the_climb_target():
    node, services = _nav_with_route()
    node._phase = MissionPhase.TAKEOFF
    climb = node._takeoff_alt
    r = services.call("set_flight_altitude", alt_m=8.0)
    assert r.success and r.data["applied_now"] is False
    assert node._takeoff_alt == climb, "moving the goalpost mid-climb could strand TAKEOFF"
    assert all(w.alt_m == 8.0 for w in node._mission.waypoints), "legs after it take the new height"


def test_flight_altitude_during_hover_never_moves_the_hold():
    node, services = _nav_with_route()
    node._phase = MissionPhase.HOVER
    node._current_wp = 2
    before = [w.alt_m for w in node._mission.waypoints]
    node._last_goto_wp = 2
    r = services.call("set_flight_altitude", alt_m=3.0)
    assert r.success and r.data["applied_now"] is False
    assert [w.alt_m for w in node._mission.waypoints] == before
    assert node._last_goto_wp == 2
    assert node._drop_climb_alt == 3.0, "no drop running: the climb-back adopts it"


def test_flight_altitude_leaves_a_running_drop_alone():
    node, services = _nav_with_route()
    node._phase = MissionPhase.HOVER
    climb_before = node._drop_climb_alt
    node._drop_running = lambda: True
    services.call("set_flight_altitude", alt_m=3.0)
    assert node._drop_climb_alt == climb_before


def test_flight_altitude_never_commands_under_pilot_override():
    node, services = _nav_with_route()
    node._phase = MissionPhase.NAVIGATE
    node._pilot_override = True
    node._last_goto_wp = 0
    before = [w.alt_m for w in node._mission.waypoints]
    r = services.call("set_flight_altitude", alt_m=4.0)
    assert r.success and r.data["applied_now"] is False
    assert [w.alt_m for w in node._mission.waypoints] == before
    assert node._last_goto_wp == 0
    assert node._cruise_alt == 4.0, "stored for the next mission"


def test_flight_altitude_is_published_in_mission_state():
    from drone_stack.bus.topics import Topics
    bus, services = MessageBus(), ServiceRegistry()
    node = NavigationNode(bus, Config.load(), services)
    node._alt_ceiling = 10.0
    seen = []
    bus.subscribe(Topics.MISSION_STATE, seen.append, deliver_latched=False)
    assert services.call("set_flight_altitude", alt_m=6.0).success
    node._publish_state()
    assert seen and seen[-1].flight_alt_m == 6.0
