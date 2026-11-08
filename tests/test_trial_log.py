"""Pre-registered experiment logging (evidence/protocol.yaml + trial_log.py)."""
from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("trial_log", ROOT / "scripts" / "trial_log.py")
tl = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(tl)

SPECS = tl.load_protocol()


@pytest.mark.parametrize("exp", sorted(SPECS))
def test_every_pass_line_refers_to_real_columns(exp):
    spec = SPECS[exp]
    cols = set(spec["columns"])
    assert spec["group"] in cols
    assert spec["checks"], "an experiment without a pass line is not pre-registered"
    for c in spec["checks"]:
        assert c["kind"] in {"rate", "ratio", "count", "value", "curve"}
        for key in ("column", "num", "den"):
            if key in c:
                assert c[key] in cols, (exp, c)
        assert c.get("min_n", 1) >= 1


def test_wilson_says_ten_out_of_ten_is_not_certainty():
    lo, hi = tl.wilson(10, 10)
    assert hi == pytest.approx(1.0) and 0.70 < lo < 0.74


def _log(tmp_path, exp, rows, sha="abc"):
    for r in rows:
        tl.add_trial(exp, r, SPECS[exp], root=tmp_path, sha=sha,
                     commit="deadbee", operator="test")
    return tl.read_trials(exp, tmp_path)


def test_too_few_runs_is_never_a_pass(tmp_path):
    rows = _log(tmp_path, "exp5", [dict(ending="full_handshake", correct_ending="yes",
                                        released="yes", lock_to_release_s=12,
                                        release_agl_m=1.0, parcel_to_person_m=1.8)] * 9)
    verdicts = {(v["check"], v["group"]): v["verdict"] for v in tl.judge(SPECS["exp5"], rows)}
    assert verdicts[("rate", "full_handshake")] == "NEED MORE"


def test_one_wrong_release_fails_the_handover(tmp_path):
    ok = dict(ending="no_person", correct_ending="yes", released="no",
              lock_to_release_s="", release_agl_m="", parcel_to_person_m="")
    bad = dict(ok, correct_ending="no", released="yes")
    rows = _log(tmp_path, "exp5", [ok] * 9 + [bad])
    v = [x for x in tl.judge(SPECS["exp5"], rows)
         if x["group"] == "no_person"]
    assert {x["verdict"] for x in v} == {"FAIL"}


def test_ten_correct_endings_pass_with_an_honest_interval(tmp_path):
    rows = _log(tmp_path, "exp6", [dict(people=2, pick="right")] * 19 + [dict(people=2, pick="declined")])
    (v,) = tl.judge(SPECS["exp6"], rows)
    assert v["verdict"] == "PASS" and v["ci"].startswith("[0.0%")


def test_moving_a_pass_line_after_the_data_is_flagged(tmp_path):
    rows = _log(tmp_path, "exp6", [dict(people=3, pick="right")] * 20, sha="old")
    assert tl.drift(rows, "new") == ["old"]
    assert "changed after data came in" in tl.render("exp6", SPECS["exp6"], rows, "new")


def test_unknown_columns_are_refused(tmp_path):
    with pytest.raises(ValueError):
        tl.add_trial("exp6", {"people": 2, "pick": "right", "colour": "red"},
                     SPECS["exp6"], root=tmp_path, sha="x", commit="x", operator="x")


def test_the_value_check_bounds_every_row(tmp_path):
    rows = _log(tmp_path, "exp7", [dict(approach="head_on", closest_m=2.1,
                                        pilot_intervened="no")] * 9 +
                [dict(approach="head_on", closest_m=1.2, pilot_intervened="no")])
    v = next(x for x in tl.judge(SPECS["exp7"], rows) if x["check"] == "value")
    assert v["verdict"] == "FAIL"


def test_the_capture_check_passes_only_on_the_fixed_layout(tmp_path):
    rows = _log(tmp_path, "exp2", [dict(layout="before_fix", image="a.img", hits=3),
                                   dict(layout="after_fix", image="b.img", hits=0)])
    (v,) = tl.judge(SPECS["exp2"], rows)
    assert v["group"] == "after_fix" and v["verdict"] == "PASS"


def test_add_refuses_until_the_protocol_is_frozen(tmp_path, monkeypatch):
    monkeypatch.setattr(tl, "FROZEN", tmp_path / "PREREGISTERED.txt")
    monkeypatch.setattr(tl, "EVIDENCE", tmp_path)
    assert tl.main(["add", "exp6", "people=2", "pick=right"]) == 2
    assert not (tmp_path / "trials").exists()
