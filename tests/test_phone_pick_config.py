"""The config that flies is the config the wrong-person figures were measured on.

By 2026-10-08 the YAML still said ambiguity_pick_prob 0.9, lacked the
leave-one-out / confirm / probe-mode keys, and claimed "below this the
camera's own pick stands" - while the code and scripts/sim_phone_pick.py had
moved to 0.95 + LOO 0.9 + a confirming look. Any figure quoted from the sim
is only true for the config that matches it, so pin the two together.
"""
from __future__ import annotations

import importlib.util
import inspect
from pathlib import Path

import pytest

from drone_stack.utils.config import Config

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location(
    "sim_phone_pick", ROOT / "scripts" / "sim_phone_pick.py")
sim = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sim)


@pytest.mark.parametrize("path", [None, "config/real.yaml"])
def test_flown_acceptance_rule_matches_the_simulated_one(path):
    d = (Config.load(str(ROOT / path)) if path else Config.load())["delivery"]
    defaults = inspect.signature(sim.adaptive).parameters
    assert d["ambiguity_pick_prob"] == defaults["pick"].default
    assert d["ambiguity_loo_prob"] == defaults["loo"].default
    assert d["ambiguity_confirm"] is defaults["confirm"].default
    assert d["ambiguity_probe_mode"] == "adaptive"
    assert d["ambiguity_probe_budget_s"] == sim.BUDGET
    assert d["locator_robust_nu"] == sim.ROBUST["robust_nu"]
    assert d["locator_gps_outlier_frac"] == sim.ROBUST["gps_outlier_frac"]
    assert d["locator_gps_outlier_sigma_m"] == sim.ROBUST["gps_outlier_sigma_m"]


def test_the_ble_geofence_has_one_value_everywhere():
    # 5 m (never passed on 2026-10-04: two receivers disagree by 5.3-6.7 m)
    # was replaced by 12 m in config, but notes kept quoting both. The
    # peripheral's fallback (GCS unreachable) and the hub's default must be
    # the configured value, or the radius depends on which process answered.
    pytest.importorskip("bluez_peripheral")
    from drone_stack.ble_handshake import drone_ble_peripheral as ble
    configured = {Config.load()["delivery"]["ble_geofence_m"],
                  Config.load(str(ROOT / "config/real.yaml"))["delivery"]["ble_geofence_m"]}
    hub_src = (ROOT / "drone_stack/gcs/hub.py").read_text()
    assert configured == {ble.GEOFENCE_MICRO_RADIUS_M}
    assert f'ble_geofence_m", {ble.GEOFENCE_MICRO_RADIUS_M})' in hub_src
