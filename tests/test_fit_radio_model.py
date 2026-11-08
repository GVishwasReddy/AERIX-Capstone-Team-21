"""fit_radio_model.py recovers a known radio from a simulated calibration
flight, and its output drives sim_phone_pick.py --radio end to end."""
from __future__ import annotations

import csv
import importlib.util
import json
import math
import random
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def _load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


fr = _load("fit_radio_model")
sim = _load("sim_phone_pick")

TRUE = dict(ref=-61.0, n=2.4, shadow=3.5, noise=5.0)
PHONE = (3.0, 18.0)


def _flight(path: Path, *, seed=3, radii=(1.5, 2.5, 4.0, 6.0, 8.0), alt=4.0, per=12):
    """An orbit at several radii round the phone, as logs/rssi writes it."""
    rng = random.Random(seed)
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "order_id", "rssi_dbm", "east_m", "north_m", "alt_m",
                    "yaw_rad", "phase", "scan_stage"])
        t = 1000.0
        for r in radii:
            for k in range(12):
                a = 2 * math.pi * k / 12
                e, n = PHONE[0] + r * math.sin(a), PHONE[1] + r * math.cos(a)
                d = math.sqrt(r * r + (alt - 1.2) ** 2)
                shadow = rng.gauss(0, TRUE["shadow"])
                for _ in range(per):
                    v = TRUE["ref"] - 10 * TRUE["n"] * math.log10(d) + shadow \
                        + rng.gauss(0, TRUE["noise"])
                    w.writerow([f"{t:.3f}", "O1", f"{v:.0f}", f"{e:.2f}", f"{n:.2f}",
                                f"{alt:.2f}", "0", "HOVER", "probe"])
                    t += 0.25
    return path


def test_it_recovers_a_known_radio(tmp_path):
    rows = fr.load([str(_flight(tmp_path / "rssi.csv"))])
    f = fr.fit(fr.bins_of(rows, PHONE, 1.2, 0.75))
    assert f["ref_dbm"] == pytest.approx(TRUE["ref"], abs=3.0)
    assert f["path_loss_n"] == pytest.approx(TRUE["n"], abs=0.35)
    assert f["rssi_sigma_db"] == pytest.approx(TRUE["noise"], abs=0.6)   # RSSI is integer-rounded
    assert f["shadow_sigma_db"] == pytest.approx(TRUE["shadow"], abs=1.5)


def test_the_true_values_sit_inside_the_bootstrap_interval(tmp_path):
    rows = fr.load([str(_flight(tmp_path / "rssi.csv", seed=9))])
    ci = fr.bootstrap(fr.bins_of(rows, PHONE, 1.2, 0.75))
    lo, hi = ci["path_loss_n"]
    assert lo <= TRUE["n"] <= hi
    lo, hi = ci["ref_dbm"]
    assert lo <= TRUE["ref"] <= hi


def test_one_distance_only_is_refused_not_guessed(tmp_path):
    rows = fr.load([str(_flight(tmp_path / "rssi.csv", radii=(4.0,)))])
    b = fr.bins_of(rows, PHONE, 1.2, 0.75)
    b = [(round(x, 1), y, c, ss) for x, y, c, ss in b]   # identical log-distances
    b = [(b[0][0], y, c, ss) for _, y, c, ss in b]
    with pytest.raises(ValueError):
        fr.fit(b)


def test_the_cli_writes_a_fit_the_simulation_accepts(tmp_path, capsys):
    out = tmp_path / "fit.json"
    assert fr.main([str(_flight(tmp_path / "rssi.csv")), "--phone-en", *map(str, PHONE),
                    "--out", str(out)]) == 0
    fit = json.loads(out.read_text())
    assert "locator_path_loss_n" in capsys.readouterr().out
    radio = sim.measured_radio(fit)
    assert radio["n"] == fit["path_loss_n"] and radio["block"] == sim.RADIOS["harsh"]["block"]
    rng = random.Random(1)
    world = sim.World(2, radio, rng)
    assert -80 < world.A < -40                      # drawn round the fitted ref


def test_latlon_input_matches_metres(tmp_path):
    from drone_stack.utils.geometry import enu_to_geodetic
    home = (12.9716, 77.5946)
    lat, lon = enu_to_geodetic(PHONE[0], PHONE[1], *home)
    path = str(_flight(tmp_path / "rssi.csv"))
    out_m, out_ll = tmp_path / "m.json", tmp_path / "ll.json"
    fr.main([path, "--phone-en", *map(str, PHONE), "--out", str(out_m)])
    fr.main([path, "--phone-latlon", str(lat), str(lon),
             "--home-latlon", *map(str, home), "--out", str(out_ll)])
    a, b = json.loads(out_m.read_text()), json.loads(out_ll.read_text())
    assert a["path_loss_n"] == pytest.approx(b["path_loss_n"], abs=0.05)
