"""Obstacle avoidance in EVERY phase of a delivery (operator requirement,
2026-10-05): at the waypoint, descending to scan height, circling for the
recipient, locked on, the precision-drop descent, the 1 m settle, the climb
back out - and, with the drop window, a fixed 30 s stay after the lock.

Each case puts the aircraft in that phase through the real stage code, drops
a static obstacle 0.6 m off the right side (well inside avoidance_stop_m) and
requires a "velocity" command moving it AWAY (left), with no goto fighting it.
NAVIGATE and the RTL return leg have their own tests
(test_waypoint_avoidance.py, test_rtl_pi_return.py).
"""
from __future__ import annotations

import time

from drone_stack.msg import ObstacleArray

from test_precision_drop import (
    _age, _arrive, _at, _commands, _descend_to, _locked, _nav, _obstacle,
    _start, _tick,
)


def _assert_moves_away(node, sent, tick):
    node._obstacles = ObstacleArray(obstacles=[_obstacle(0.6, 90.0)])
    sent.clear()
    tick()
    vel = _commands(sent, "velocity")
    assert vel, "no avoidance response in this phase"
    assert vel[-1].params["vy"] < 0.0, "did not move away from the obstacle"
    assert not _commands(sent, "goto"), "the phase fought the avoidance"


def test_descending_to_scan_height():
    node, sent, *_ = _nav()
    node._enter_hover(node._mission.waypoints[1])
    _at(node, 20.0, alt=5.0)                       # still above scan height
    assert node._scan_stage == "descend"
    _assert_moves_away(node, sent, node._do_scan)


def test_circling_for_the_recipient():
    node, sent, *_ = _nav()
    node._enter_hover(node._mission.waypoints[1])
    _at(node, 20.0, alt=4.0)
    node._do_scan()
    assert node._scan_stage == "search"
    node._scan_next_step = 0.0                     # an orbit step is due
    _assert_moves_away(node, sent, node._do_scan)


def test_locked_on_the_recipient():
    node, sent, *_ = _locked()
    _assert_moves_away(node, sent, node._do_scan)
    assert node._scan_stage == "locked"


def test_drop_approach_and_descent():
    node, sent, *_ = _start()
    _assert_moves_away(node, sent, lambda: _tick(node, agl=4.0))
    node._obstacles = ObstacleArray(obstacles=[])
    _tick(node, agl=4.0)
    _arrive(node)
    _assert_moves_away(node, sent, lambda: _tick(node, agl=4.0))


def test_at_one_metre_settling_for_the_release():
    node, sent, results, _ = _start()
    _arrive(node)
    _descend_to(node, 1.0)
    assert node._drop.stage == "settle"
    _assert_moves_away(node, sent, lambda: _tick(node, agl=1.0))
    _age(node, 2.0)
    _tick(node, agl=1.0)
    assert not results, "released the parcel while dodging"


def test_climbing_out_after_the_release():
    node, sent, *_ = _start()
    _arrive(node)
    _descend_to(node, 1.0)
    _age(node, 2.0)
    _tick(node, agl=1.0)                           # release
    _age(node, 3.0)
    _tick(node, agl=1.0)                           # relock -> climb
    assert node._drop.stage == "climb"
    _assert_moves_away(node, sent, lambda: _tick(node, agl=1.5))


# --------------------------------------------------------------------------- #
# the 30 s stay after the lock
# --------------------------------------------------------------------------- #
def test_the_locked_stay_is_a_fixed_30s_even_with_the_phone_talking():
    """real.yaml: handshake_window_s 30, handshake_phone_grace_s 0. A phone
    still on the link (failing the geofence, say) must not hold the aircraft
    past the 30 s - it returns home."""
    node, *_ = _locked(handshake_window_s=30.0, handshake_phone_grace_s=0.0)
    now = time.monotonic()
    assert 29.0 < node._hover_until - now <= 30.0
    node._phone_first_at = node._phone_seen_at = now      # phone on the link
    node._hover_until = now - 0.01                        # 30 s have passed
    node._do_scan()
    assert node._scan_stage == "", "the phone kept the aircraft past 30 s"
    assert node._phase.name == "RTL"


def test_a_drop_under_way_at_30s_is_not_cut_off():
    node, sent, results, _ = _start(handshake_window_s=30.0,
                                    handshake_phone_grace_s=0.0)
    node._hover_until = time.monotonic() - 5.0            # window long gone
    _arrive(node)
    _descend_to(node, 1.0)
    _age(node, 2.0)
    _tick(node, agl=1.0)
    assert results and results[-1].success, "the drop was abandoned at 30 s"
