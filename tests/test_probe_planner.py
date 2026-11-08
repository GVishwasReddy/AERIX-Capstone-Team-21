"""Adaptive RSSI probe + the locator's robustness checks (2026-10-08).

The planner chooses where to listen so the people below can be told apart;
the locator decides whether what was heard is enough. Both are pinned on
geometry whose answer is known in closed form, not on tuned numbers."""
from __future__ import annotations

import math
import random

import numpy as np

from drone_stack.nodes import probe_planner as pp
from drone_stack.nodes.phone_locator import PhoneLocator

M = pp.RadioModel()
A, B = (2.0, 0.0), (-2.0, 0.0)          # two people 4 m apart, east-west


def _rssi(pos, phone, alt=3.0, ref=-58.0, n=2.0):
    d = math.sqrt((pos[0] - phone[0]) ** 2 + (pos[1] - phone[1]) ** 2 + (alt - 1.2) ** 2)
    return ref - 10 * n * math.log10(d)


# -- the separation measure -------------------------------------------------

def test_no_bins_no_separation():
    d2 = pp.separation(M, np.zeros((0, 3)), np.zeros(0), np.array([A, B]))
    assert d2[0, 1] == 0.0


def test_a_common_offset_buys_nothing():
    # One bin alone cannot separate: the unknown 1 m level A absorbs it.
    # It takes a CONTRAST between two vantage points.
    one = pp.separation(M, np.array([[3.0, 0.0, 3.0]]), np.array([8.0]), np.array([A, B]))
    two = pp.separation(M, np.array([[3.0, 0.0, 3.0], [-3.0, 0.0, 3.0]]),
                        np.array([8.0, 8.0]), np.array([A, B]))
    assert one[0, 1] < 0.1 * two[0, 1]


def test_the_bisector_is_worthless():
    # Every point on the perpendicular bisector is equidistant from A and B.
    d2 = pp.separation(M, np.array([[0.0, 1.0, 3.0], [0.0, -2.0, 3.0]]),
                       np.array([8.0, 8.0]), np.array([A, B]))
    assert d2[0, 1] < 1e-9


# -- choosing the next vantage ----------------------------------------------

def test_next_vantage_goes_where_the_people_differ():
    bins = [(0.0, 0.0, 3.0, -60.0, 8)]           # heard once, over the middle
    feas = [(0.0, 3.0), (0.0, -3.0), (3.5, 0.0), (-3.5, 0.0)]
    v = pp.next_vantage(M, [A, B], [0.5, 0.5], bins, feas, (0.0, 0.0), 3.0)
    assert abs(v.north) < 1e-9 and abs(v.east) == 3.5   # off the bisector


def test_a_new_spot_beats_a_second_dwell_on_an_old_one():
    # Shadowing is shared within a bin: more samples there are worth little.
    bins = [(3.5, 0.0, 3.0, -60.0, 8), (0.0, 0.0, 3.0, -62.0, 8)]
    feas = [(3.5, 0.0), (-3.5, 0.0)]
    v = pp.next_vantage(M, [A, B], [0.5, 0.5], bins, feas, (3.5, 0.0), 3.0)
    assert v.east == -3.5


def test_nothing_reachable_nothing_planned():
    assert pp.next_vantage(M, [A, B], [0.5, 0.5], [], [], (0, 0), 3.0) is None
    assert pp.next_vantage(M, [A], [1.0], [], [(1, 1)], (0, 0), 3.0) is None


def test_feasible_points_stay_near_the_order():
    pts = pp.feasible_points([A, B], (0.0, 0.0), 3.0)
    assert pts and all(math.hypot(e, n) <= 3.0 + 1e-9 for e, n in pts)


def test_confusion_bound_falls_as_separation_grows():
    near = pp.confusion_bound([0.5, 0.5], np.array([[0, 1.0], [1.0, 0]]))
    far = pp.confusion_bound([0.5, 0.5], np.array([[0, 40.0], [40.0, 0]]))
    assert far < near <= 0.5


# -- the locator: leave-one-out and heavy tails ------------------------------

def _listen(loc, phone, pts, *, t0=1000.0, per=8, sigma=1.0, seed=3, alt=3.0):
    rng = random.Random(seed)
    t = t0
    for p in pts:
        for _ in range(per):
            loc.add_rssi(_rssi(p, phone, alt) + rng.gauss(0, sigma), p[0], p[1], alt, t)
            t += 0.25
    return t


def test_a_pick_resting_on_the_gps_alone_fails_leave_one_out():
    loc = PhoneLocator(gps_sigma_m=1.0)
    loc.add_gps(*A, 1000.0)                         # GPS says A, firmly
    t = _listen(loc, A, [(0.0, 3.0), (0.0, -3.0)])  # bisector only: RSSI says nothing
    a = loc.assess_candidates([A, B], 0.0, 0.0, t)
    assert a.top == 0 and a.prob > 0.9
    assert a.loo_min < 0.9                          # remove the GPS and it is a coin


def test_well_spread_evidence_passes_leave_one_out():
    loc = PhoneLocator()
    ring = [(3.5 * math.cos(k * math.pi / 5), 3.5 * math.sin(k * math.pi / 5)) for k in range(10)]
    t = _listen(loc, A, ring)
    a = loc.assess_candidates([A, B], 0.0, 0.0, t)
    assert a.top == 0 and a.prob > 0.95 and a.loo_min > 0.9


def test_one_blocked_bin_cannot_flip_a_robust_pick():
    pts = [(3.5, 0.0), (-3.5, 0.0), (0.0, 3.0), (2.0, 2.0), (-2.0, -2.0)]
    def run(**kw):
        loc = PhoneLocator(**kw)
        t = _listen(loc, A, pts)
        for _ in range(8):                          # a hand over the phone, right above A
            loc.add_rssi(_rssi((2.0, 0.5), A) - 25.0, 2.0, 0.5, 3.0, t)
            t += 0.25
        return loc.assess_candidates([A, B], 0.0, 0.0, t)
    gauss, robust = run(), run(robust_nu=4.0)
    assert robust.top == 0
    assert robust.probs[0] > gauss.probs[0]


def test_a_biased_gps_fix_cannot_outvote_the_rssi_with_heavy_tails():
    pts = [(3.5, 0.0), (-3.5, 0.0), (0.0, 3.0)]
    def run(**kw):
        loc = PhoneLocator(gps_sigma_m=2.0, **kw)
        loc.add_gps(-6.0, 0.0, 1000.0)              # multipath: 8 m off, toward B
        t = _listen(loc, A, pts)
        return loc.assess_candidates([A, B], 0.0, 0.0, t)
    assert run(gps_outlier_frac=0.2, gps_outlier_sigma_m=15.0).probs[0] > run().probs[0]


def test_defaults_are_the_original_gaussian_model():
    loc = PhoneLocator()
    assert loc.robust_nu == 0.0 and loc.gps_outlier_frac == 0.0


def test_listening_again_from_the_same_spot_is_worth_little():
    # Without the shared-shadowing merge, a second dwell over (3.5, 0) would
    # count as a fresh independent bin and look as good as a new spot.
    bins = [(3.5, 0.0, 3.0, -60.0, 8), (-3.5, 0.0, 3.0, -66.0, 8)]
    again = pp.next_vantage(M, [A, B], [0.5, 0.5], bins, [(3.5, 0.0)], (3.5, 0.0), 3.0)
    fresh = pp.next_vantage(M, [A, B], [0.5, 0.5], bins, [(3.5, 1.0)], (3.5, 0.0), 3.0)
    assert again.gain_per_s < 0.5 * fresh.gain_per_s
