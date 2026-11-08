#!/usr/bin/env python3
"""Which of N people holds the phone? Hover vs fixed ring vs adaptive probe.

    scripts/sim_phone_pick.py                     # 2 and 3 people, both radios
    scripts/sim_phone_pick.py --trials 2000
    scripts/sim_phone_pick.py --radio evidence/radio_fit_20261012.json

Monte Carlo through the REAL PhoneLocator and probe_planner, with the same
acceptance rule the navigator flies (navigation_node._scan_lock_accepted), so
it measures the code that flies rather than a re-implementation of it.

Each trial drops N people at least 0.8*sep apart (sep 3 m) round a drop
point, puts the phone on one, gives the locator the scan orbit's evidence
(4 points, 1 s each) and one phone-GPS fix, then runs a policy:

  hover     stay over the group and listen (the "wait longer" baseline)
  ring      the fixed probe: 6 points round the group + 1.5 m past each person
            Gaussian model, accept at posterior >= 0.9
  adaptive  each next vantage chosen by probe_planner; heavy-tailed model;
            accept only on posterior >= 0.95 AND leave-one-out >= 0.9, and
            only if the same person still passes after one confirming look

All three get the same flight budget (36 s, 1 m/s legs, 2 s dwell).

Two radios. "nominal" is the locator's own model. "harsh" is deliberately
NOT: shadowing 6.5 dB (model 4.5), noise 8 dB (6), path-loss exponent 2.6
(2.0), the phone's 1 m level spread +/-8 dB, 12% of dwells blocked by a hand
or body (-12 dB), 25% of GPS fixes multipath-biased 5-9 m, and the camera's
ground positions off by 0.5 m. A policy that is only safe when its model is
right is not safe.

  identified  accepted AND the right person
  wrong       accepted AND someone else - a parcel handed to a stranger
  the rest    declined: the aircraft returns with the parcel

The radio figures are literature values for 2.4 GHz near bodies, not yet
re-fit to this aircraft's logs/rssi. This is a model, not a flight.

--radio <fit.json> (from scripts/fit_radio_model.py) adds a third radio,
"measured": the fitted ref/path-loss/sigmas, keeping the harsh radio's body
blocking, GPS multipath and camera error (a calibration flight does not
measure those). The locator then scores with the fitted values too, as the
navigator does once they are in config.
"""
from __future__ import annotations

import argparse
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from drone_stack.nodes import probe_planner as pp  # noqa: E402
from drone_stack.nodes.phone_locator import PhoneLocator  # noqa: E402

ALT, RATE, SPEED, DWELL, BUDGET, MAX_OFF = 3.0, 4.0, 1.0, 2.0, 36.0, 6.0
DROP = (0.0, 20.0)
RADIOS = {
    "nominal": dict(shadow=4.5, noise=6.0, n=2.0, aspread=0, block=0.0, gpsbias=0.0, camerr=0.0),
    "harsh":   dict(shadow=6.5, noise=8.0, n=2.6, aspread=8, block=0.12, gpsbias=0.25, camerr=0.5),
}
# What the navigator configures (config/default.yaml delivery: locator_*).
ROBUST = dict(robust_nu=4.0, gps_outlier_frac=0.2, gps_outlier_sigma_m=15.0)
#: Locator model overrides; set from a --radio fit (the locator_* config keys).
LOCATOR_KW: dict = {}


def measured_radio(fit: dict) -> dict:
    """A world drawn from this aircraft's fitted radio (fit_radio_model.py)."""
    return dict(RADIOS["harsh"], ref=fit["ref_dbm"], n=fit["path_loss_n"],
                shadow=fit["shadow_sigma_db"], noise=fit["rssi_sigma_db"], aspread=4)


class World:
    def __init__(self, n, radio, rng):
        self.rng, self.r = rng, radio
        while True:
            ps = [(DROP[0] + rng.uniform(-3, 3), DROP[1] + rng.uniform(-3, 3)) for _ in range(n)]
            if all(math.dist(ps[i], ps[j]) >= 2.4 for i in range(n) for j in range(i)):
                break
        self.people, self.truth = ps, rng.randrange(n)
        self.seen = [(x + rng.gauss(0, radio["camerr"]), y + rng.gauss(0, radio["camerr"]))
                     for x, y in ps]
        self.A = radio.get("ref", -58.0) + rng.uniform(-radio["aspread"], radio["aspread"])
        bias = rng.uniform(5, 9) if rng.random() < radio["gpsbias"] else 0.0
        ang = rng.uniform(0, 2 * math.pi)
        px, py = ps[self.truth]
        self.gps = (px + rng.gauss(0, 5) + bias * math.cos(ang),
                    py + rng.gauss(0, 5) + bias * math.sin(ang))

    def listen(self, loc, p, now, secs):
        r, ph = self.r, self.people[self.truth]
        shadow = self.rng.gauss(0, r["shadow"]) - (12 if self.rng.random() < r["block"] else 0)
        d = math.sqrt((p[0] - ph[0]) ** 2 + (p[1] - ph[1]) ** 2 + (ALT - 1.2) ** 2)
        for _ in range(int(secs * RATE)):
            rssi = self.A - 10 * r["n"] * math.log10(max(d, 0.3)) + shadow + self.rng.gauss(0, r["noise"])
            loc.add_rssi(rssi, p[0], p[1], ALT, now)
            now += 1 / RATE
        return now


def start(world, **kw):
    loc, now = PhoneLocator(**{**LOCATOR_KW, **kw}), 1000.0
    loc.add_gps(*world.gps, now)
    for k in range(4):                                   # the scan orbit so far
        p = (DROP[0] + 2 * math.sin(k * math.pi / 2), DROP[1] + 2 * math.cos(k * math.pi / 2))
        now = world.listen(loc, p, now, 1.0)
    return loc, now


def hover(world):
    loc, now = start(world)
    c = (sum(x for x, _ in world.seen) / len(world.seen), sum(y for _, y in world.seen) / len(world.seen))
    now = world.listen(loc, c, now, BUDGET)
    a = loc.assess_candidates(world.seen, *DROP, now)
    return a.prob >= 0.9, a.top, BUDGET


def ring(world):
    loc, now = start(world)
    n = len(world.seen)
    cx, cy = sum(x for x, _ in world.seen) / n, sum(y for _, y in world.seen) / n
    pts = [(cx + 2 * math.sin(2 * math.pi * k / 6), cy + 2 * math.cos(2 * math.pi * k / 6)) for k in range(6)]
    for x, y in world.seen:
        d = math.hypot(x - cx, y - cy)
        if d > 0.3:
            pts.append((cx + (x - cx) * (d + 1.5) / d, cy + (y - cy) * (d + 1.5) / d))
    here, t = DROP, 0.0
    for p in pts:
        cost = math.dist(here, p) / SPEED + DWELL
        if t + cost > BUDGET:
            break
        t, here = t + cost, p
        now = world.listen(loc, p, now, DWELL)
    a = loc.assess_candidates(world.seen, *DROP, now)
    return a.prob >= 0.9, a.top, t


def adaptive(world, pick=0.95, loo=0.9, confirm=True):
    loc, now = start(world, **ROBUST)
    model = pp.RadioModel.of(loc)
    feas = pp.feasible_points(world.seen, DROP, MAX_OFF)
    here, t = DROP, 0.0
    sure = lambda a: a.prob >= pick and a.loo_min >= loo  # noqa: E731

    def fly(probs):
        nonlocal here, t, now
        v = pp.next_vantage(model, world.seen, probs, loc.bins(now), feas, here, ALT, dwell_s=DWELL)
        if v is None or v.gain_per_s <= 1e-4:
            return False
        cost = math.dist(here, (v.east, v.north)) / SPEED + DWELL
        if t + cost > BUDGET:
            return False
        t, here = t + cost, (v.east, v.north)
        now = world.listen(loc, here, now, DWELL)
        return True

    while True:
        a = loc.assess_candidates(world.seen, *DROP, now)
        if sure(a) or not fly(a.probs):
            break
    if not sure(a):
        return False, a.top, t
    if confirm:
        first = a.top
        t_budget, t_saved = BUDGET, t
        globals()["BUDGET"] = t + 6 + 2 * DWELL         # the confirm look is always allowed
        fly([1 / len(world.seen)] * len(world.seen))
        globals()["BUDGET"] = t_budget
        a = loc.assess_candidates(world.seen, *DROP, now)
        return sure(a) and a.top == first, a.top, max(t, t_saved)
    return True, a.top, t


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--trials", type=int, default=600)
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--radio", type=Path, help="fit JSON from scripts/fit_radio_model.py")
    a = ap.parse_args()
    radios = ["nominal", "harsh"]
    if a.radio:
        import json
        fit = json.loads(a.radio.read_text())
        RADIOS["measured"] = measured_radio(fit)
        LOCATOR_KW.update(path_loss_n=fit["path_loss_n"], ref_dbm=fit["ref_dbm"],
                          rssi_sigma_db=fit["rssi_sigma_db"],
                          shadow_sigma_db=fit["shadow_sigma_db"])
        radios = ["measured"]
    for radio in radios:
        for n in (2, 3):
            for name, policy in (("hover", hover), ("ring", ring), ("adaptive", adaptive)):
                rng = random.Random(a.seed)
                res = []
                for _ in range(a.trials):
                    w = World(n, RADIOS[radio], rng)
                    acc, top, secs = policy(w)
                    res.append((acc, top == w.truth, secs))
                ok = sum(1 for acc, right, _ in res if acc and right) / a.trials
                bad = sum(1 for acc, right, _ in res if acc and not right) / a.trials
                secs = sum(s for *_, s in res) / a.trials
                print(f"{radio:8s} {n} people  {name:9s} identified {ok:6.1%}  "
                      f"wrong {bad:5.2%}  declined {1 - ok - bad:6.1%}  flight {secs:4.1f}s",
                      flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
