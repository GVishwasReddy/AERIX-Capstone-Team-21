#!/usr/bin/env python3
"""Fit the phone-RSSI radio model to THIS aircraft's logs.

    scripts/fit_radio_model.py logs/rssi/rssi-20261012.csv \\
        --phone-latlon 12.971601 77.594612 --home-latlon 12.971550 77.594580 \\
        --out evidence/radio_fit_20261012.json

    # or, if you measured the phone's spot in metres from the launch point:
    scripts/fit_radio_model.py logs/rssi/rssi-*.csv --phone-en 3.2 18.7

Why: the person-picking claim (docs/patent/CLAIM_CHART.md §0) rests on
scripts/sim_phone_pick.py, which still draws its radio from literature
values for 2.4 GHz near bodies. Fit those values here, then re-run the
simulation with them (`sim_phone_pick.py --radio <out.json>`) and put them in
config (locator_* keys, printed below). A model checked against your own
aircraft's measurements is evidence; a textbook value is not.

Calibration flight (evidence/FLIGHT_DAY_RUNSHEET.md, "radio calibration"):
one person, phone in hand at a MARKED spot (tape-measured from the launch
point, or its lat/lon averaged on a second phone), drone flying the scan
orbit plus a few passes 2-8 m away at the scan altitude. Nobody else with
Bluetooth near the spot.

Model, the same one PhoneLocator scores with:
    rssi = A - 10 n log10(d) + shadow_bin + noise
    d = 3-D distance from the aircraft to the phone (held phone_height_m up)
Samples are grouped into position bins (bin_m, as the locator does):
    noise sigma  = pooled within-bin standard deviation
    A, n         = least squares on bin means vs log10(d), weighted by count
    shadow sigma = spread of bin means about the fit, minus the share the
                   noise explains
95% intervals come from a bootstrap over bins.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from drone_stack.utils.geometry import geodetic_to_enu  # noqa: E402


def load(paths: list[str], order: str | None = None) -> list[dict]:
    rows = []
    for pattern in paths:
        for p in sorted(glob.glob(pattern)) or [pattern]:
            with open(p, newline="", encoding="utf-8") as fh:
                for r in csv.DictReader(fh):
                    if order and r.get("order_id") != order:
                        continue
                    rows.append({"rssi": float(r["rssi_dbm"]), "e": float(r["east_m"]),
                                 "n": float(r["north_m"]), "alt": float(r["alt_m"])})
    return rows


def bins_of(rows, phone_en, phone_h, bin_m):
    """(log10 d, mean rssi, count, within-bin sum of squares) per position bin."""
    groups: dict[tuple, list] = {}
    for r in rows:
        key = (round(r["e"] / bin_m), round(r["n"] / bin_m), round(r["alt"] / bin_m))
        groups.setdefault(key, []).append(r)
    out = []
    for g in groups.values():
        e = sum(r["e"] for r in g) / len(g)
        n = sum(r["n"] for r in g) / len(g)
        a = sum(r["alt"] for r in g) / len(g)
        d = math.sqrt((e - phone_en[0]) ** 2 + (n - phone_en[1]) ** 2
                      + max(a - phone_h, 0.3) ** 2)
        vals = [r["rssi"] for r in g]
        m = sum(vals) / len(vals)
        out.append((math.log10(max(d, 0.3)), m, len(vals), sum((v - m) ** 2 for v in vals)))
    return out


def fit(bins) -> dict:
    """Weighted least squares of bin mean on log10 d, then the two sigmas."""
    W = sum(c for _, _, c, _ in bins)
    xb = sum(c * x for x, _, c, _ in bins) / W
    yb = sum(c * y for _, y, c, _ in bins) / W
    sxx = sum(c * (x - xb) ** 2 for x, _, c, _ in bins)
    if sxx <= 1e-9:
        raise ValueError("all bins at one distance - the path-loss slope is unidentifiable; "
                         "fly passes at several distances (2-8 m)")
    slope = sum(c * (x - xb) * (y - yb) for x, y, c, _ in bins) / sxx
    A = yb - slope * xb
    n_path = -slope / 10.0
    dof_in = sum(c - 1 for _, _, c, _ in bins)
    noise_var = sum(ss for *_, ss in bins) / dof_in if dof_in > 0 else 0.0
    resid = [(y - (A + slope * x), c) for x, y, c, _ in bins]
    k = len(bins)
    between = sum(r * r for r, _ in resid) / max(k - 2, 1)
    mean_inv_c = sum(1.0 / c for _, c in resid) / k
    shadow_var = max(between - noise_var * mean_inv_c, 0.0)
    span = max(x for x, *_ in bins) - min(x for x, *_ in bins)
    return {"ref_dbm": A, "path_loss_n": n_path, "rssi_sigma_db": math.sqrt(noise_var),
            "shadow_sigma_db": math.sqrt(shadow_var), "bins": k, "samples": W,
            "distance_ratio": 10 ** span}


def bootstrap(bins, reps=400, seed=7) -> dict:
    rng = random.Random(seed)
    keys = ("ref_dbm", "path_loss_n", "rssi_sigma_db", "shadow_sigma_db")
    draws = {k: [] for k in keys}
    for _ in range(reps):
        try:
            f = fit([rng.choice(bins) for _ in bins])
        except ValueError:
            continue
        for k in keys:
            draws[k].append(f[k])
    out = {}
    for k, v in draws.items():
        v.sort()
        if v:
            out[k] = (v[int(0.025 * len(v))], v[int(0.975 * len(v)) - 1])
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("csv", nargs="+", help="logs/rssi/rssi-*.csv (globs ok)")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--phone-en", nargs=2, type=float, metavar=("EAST", "NORTH"),
                   help="phone position, metres east/north of the launch point")
    g.add_argument("--phone-latlon", nargs=2, type=float, metavar=("LAT", "LON"))
    ap.add_argument("--home-latlon", nargs=2, type=float, metavar=("LAT", "LON"),
                    help="launch point (home), needed with --phone-latlon")
    ap.add_argument("--phone-height", type=float, default=1.2)
    ap.add_argument("--bin-m", type=float, default=0.75)
    ap.add_argument("--order", default=None, help="only this order id's samples")
    ap.add_argument("--out", type=Path, help="write the fit as JSON (for sim_phone_pick --radio)")
    a = ap.parse_args(argv)

    if a.phone_latlon:
        if not a.home_latlon:
            ap.error("--phone-latlon needs --home-latlon")
        phone = geodetic_to_enu(*a.phone_latlon, *a.home_latlon)
    else:
        phone = tuple(a.phone_en)
    rows = load(a.csv, a.order)
    if len(rows) < 40:
        print(f"only {len(rows)} samples - fly longer before trusting a fit", file=sys.stderr)
        return 2
    b = bins_of(rows, phone, a.phone_height, a.bin_m)
    try:
        f = fit(b)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 2
    ci = bootstrap(b)
    print(f"{f['samples']} samples in {f['bins']} position bins; distances span "
          f"{f['distance_ratio']:.1f}x")
    for k in ("ref_dbm", "path_loss_n", "rssi_sigma_db", "shadow_sigma_db"):
        lo, hi = ci.get(k, (float("nan"), float("nan")))
        print(f"  {k:<16} {f[k]:7.2f}   95% CI [{lo:.2f}, {hi:.2f}]")
    if f["distance_ratio"] < 3:
        print("  ! distances span < 3x: path_loss_n is weakly determined - add near/far passes")
    print("\nconfig/real.yaml, delivery: (after you agree with the numbers)")
    print(f"  locator_ref_dbm: {f['ref_dbm']:.1f}\n  locator_path_loss_n: {f['path_loss_n']:.2f}\n"
          f"  locator_rssi_sigma_db: {f['rssi_sigma_db']:.1f}\n"
          f"  locator_shadow_sigma_db: {f['shadow_sigma_db']:.1f}")
    if a.out:
        a.out.write_text(json.dumps({**f, "ci95": ci, "phone_en": phone,
                                     "sources": a.csv}, indent=2), encoding="utf-8")
        print(f"\nwrote {a.out}  ->  scripts/sim_phone_pick.py --radio {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
