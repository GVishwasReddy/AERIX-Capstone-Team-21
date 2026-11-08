#!/usr/bin/env python3
"""Log experiment trials and judge them against pre-registered pass lines.

    scripts/trial_log.py freeze                       # before the first trial
    scripts/trial_log.py add exp5 ending=full_handshake correct_ending=yes \\
        released=yes lock_to_release_s=14.2 release_agl_m=1.04 \\
        parcel_to_person_m=1.9 --raw logs/ble/ble-20261012.log --note "run 3"
    scripts/trial_log.py report                       # every experiment
    scripts/trial_log.py report exp5 --md evidence/REPORT.md

Pass lines live in evidence/protocol.yaml and nowhere else. Trials append to
evidence/trials/<exp>.csv with the date, the git commit of the code that
flew, the operator, the raw log the numbers came from, and the sha256 of the
protocol they were logged under. A report marks any experiment whose protocol
changed after its first trial: moving a pass line once the data is in turns
evidence back into opinion.

Rates come with a 95% Wilson interval. With 10 runs a 10/10 result still
only bounds the true rate above ~72%; the report prints that so nobody
quotes 100% as certainty.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import getpass
import hashlib
import math
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence"
PROTOCOL = EVIDENCE / "protocol.yaml"
FROZEN = EVIDENCE / "PREREGISTERED.txt"
BASE_COLS = ["logged_at", "commit", "operator", "protocol_sha", "raw", "note"]


# ── protocol ────────────────────────────────────────────────────────────────

def protocol_sha(path: Path = PROTOCOL) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()[:16]


def load_protocol(path: Path = PROTOCOL) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))["experiments"]


def git_commit() -> str:
    try:
        out = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=5)
        dirty = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain",
                                "--untracked-files=no"],
                               capture_output=True, text=True, timeout=5).stdout.strip()
        return out.stdout.strip() + ("+dirty" if dirty else "") or "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"


# ── statistics ──────────────────────────────────────────────────────────────

def wilson(k: int, n: int, z: float = 1.959964) -> tuple[float, float]:
    """95% Wilson score interval for k successes in n trials."""
    if n == 0:
        return 0.0, 1.0
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


# ── trials ──────────────────────────────────────────────────────────────────

def trials_path(exp: str, root: Path = EVIDENCE) -> Path:
    return root / "trials" / f"{exp}.csv"


def read_trials(exp: str, root: Path = EVIDENCE) -> list[dict]:
    p = trials_path(exp, root)
    if not p.exists():
        return []
    with open(p, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def add_trial(exp: str, values: dict, spec: dict, *, raw: str = "", note: str = "",
              root: Path = EVIDENCE, sha: str | None = None,
              commit: str | None = None, operator: str | None = None) -> dict:
    missing = [c for c in spec["columns"] if c not in values]
    unknown = [c for c in values if c not in spec["columns"]]
    if missing or unknown:
        raise ValueError(f"{exp}: missing {missing}, unknown {unknown}; "
                         f"columns are {spec['columns']}")
    row = {"logged_at": dt.datetime.now().isoformat(timespec="seconds"),
           "commit": commit or git_commit(),
           "operator": operator or getpass.getuser(),
           "protocol_sha": sha or protocol_sha(), "raw": raw, "note": note}
    row.update({c: str(values[c]) for c in spec["columns"]})
    p = trials_path(exp, root)
    p.parent.mkdir(parents=True, exist_ok=True)
    new = not p.exists()
    with open(p, "a", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=BASE_COLS + list(spec["columns"]))
        if new:
            w.writeheader()
        w.writerow(row)
    return row


# ── judging ─────────────────────────────────────────────────────────────────

def _groups(rows, spec, check):
    g = spec.get("group")
    if check.get("only_group") is not None:
        return {str(check["only_group"]): [r for r in rows
                                           if str(r.get(g)) == str(check["only_group"])]}
    if check.get("per_group") or check["kind"] == "curve":
        out: dict[str, list] = {}
        for r in rows:
            out.setdefault(str(r.get(g)), []).append(r)
        return out or {"(none)": []}
    return {"all": rows}


def judge(spec: dict, rows: list[dict]) -> list[dict]:
    """One verdict per check (per group where asked): PASS, FAIL, NEED MORE
    (fewer than min_n rows) or INFO (curves)."""
    out = []
    for check in spec.get("checks", []):
        kind, min_n = check["kind"], int(check.get("min_n", 1))
        for group, rs in sorted(_groups(rows, spec, check).items()):
            n = len(rs)
            v: dict = {"check": kind, "group": group, "n": n}
            if kind in ("rate", "curve", "count"):
                k = sum(1 for r in rs if str(r.get(check["column"])) == str(check["equals"]))
                v["what"] = f'{check["column"]}={check["equals"]}'
                if kind == "count":
                    v["value"] = k
                    ok = k <= check["max"]
                    v["line"] = f"<= {check['max']}"
                else:
                    lo, hi = wilson(k, n)
                    v["value"] = f"{k}/{n} = {k / n:.1%}" if n else "0/0"
                    v["ci"] = f"[{lo:.1%}, {hi:.1%}]"
                    rate = k / n if n else 0.0
                    ok = (("min" not in check or rate >= check["min"]) and
                          ("max" not in check or rate <= check["max"]))
                    v["line"] = " and ".join(
                        f"{'>=' if b == 'min' else '<='} {check[b]:.0%}"
                        for b in ("min", "max") if b in check) or "-"
            elif kind == "ratio":
                num = sum(float(r.get(check["num"]) or 0) for r in rs)
                den = sum(float(r.get(check["den"]) or 0) for r in rs)
                rate = num / den if den else 0.0
                lo, hi = wilson(int(num), int(den))
                v.update(what=f'{check["num"]}/{check["den"]}',
                         value=f"{num:g}/{den:g} = {rate:.1%}", ci=f"[{lo:.1%}, {hi:.1%}]")
                ok = (("min" not in check or rate >= check["min"]) and
                      ("max" not in check or rate <= check["max"]))
                v["line"] = " and ".join(f"{'>=' if b == 'min' else '<='} {check[b]:.0%}"
                                         for b in ("min", "max") if b in check)
            elif kind == "value":
                vals = [float(r[check["column"]]) for r in rs
                        if str(r.get(check["column"], "")).strip() != ""]
                v["what"] = check["column"]
                v["value"] = (f"min {min(vals):g}, max {max(vals):g}" if vals else "-")
                ok = bool(vals) and (("min" not in check or min(vals) >= check["min"]) and
                                     ("max" not in check or max(vals) <= check["max"]))
                v["line"] = " and ".join(f"{'all >=' if b == 'min' else 'all <='} {check[b]:g}"
                                         for b in ("min", "max") if b in check)
            else:
                raise ValueError(f"unknown check kind {kind!r}")
            if kind == "curve":
                v["verdict"] = "INFO" if n >= min_n else "NEED MORE"
                v["line"] = f"(curve, {min_n}/point)"
            elif n < min_n:
                v["verdict"] = "NEED MORE"
            else:
                v["verdict"] = "PASS" if ok else "FAIL"
            v["need"] = min_n
            out.append(v)
    return out


def drift(rows: list[dict], sha: str) -> list[str]:
    """Protocol hashes the rows were logged under that differ from today's."""
    return sorted({r.get("protocol_sha", "") for r in rows} - {sha})


def render(exp: str, spec: dict, rows: list[dict], sha: str) -> str:
    lines = [f"### {exp} — {spec['title']}", "",
             f"Turns into: {spec.get('turns_into', '-')}. "
             f"{len(rows)} trial(s) logged."]
    moved = drift(rows, sha)
    if moved:
        lines += ["", f"> ⚠️ Pass lines changed after data came in (rows logged "
                      f"under protocol {', '.join(moved)}, now {sha}). Do not "
                      "quote a PASS from this table without saying so."]
    if rows:
        first, last = rows[0]["logged_at"][:10], rows[-1]["logged_at"][:10]
        commits = sorted({r["commit"] for r in rows})
        lines += ["", f"Dates {first} to {last}; code {', '.join(commits)}."]
    lines += ["", "| check | group | measured | 95% CI | pass line | n / need | verdict |",
              "|---|---|---|---|---|---|---|"]
    for v in judge(spec, rows):
        lines.append(f"| {v['check']} {v.get('what', '')} | {v['group']} | {v['value']} | "
                     f"{v.get('ci', '')} | {v['line']} | {v['n']} / {v['need']} | "
                     f"**{v['verdict']}** |")
    return "\n".join(lines)


# ── cli ─────────────────────────────────────────────────────────────────────

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("freeze", help="record the protocol hash before the first trial")
    a_add = sub.add_parser("add", help="append one trial: add <exp> col=value ...")
    a_add.add_argument("exp")
    a_add.add_argument("values", nargs="+", help="column=value")
    a_add.add_argument("--raw", default="", help="raw log the numbers came from")
    a_add.add_argument("--note", default="")
    a_rep = sub.add_parser("report")
    a_rep.add_argument("exps", nargs="*")
    a_rep.add_argument("--md", type=Path, help="also write the report here")
    a = ap.parse_args(argv)

    specs = load_protocol()
    sha = protocol_sha()
    if a.cmd == "freeze":
        stamp = dt.datetime.now().isoformat(timespec="seconds")
        with open(FROZEN, "a", encoding="utf-8") as fh:
            fh.write(f"{stamp}  protocol.yaml sha256[:16]={sha}  commit={git_commit()}\n")
        print(f"frozen protocol {sha} -> {FROZEN.relative_to(ROOT)}; commit it now")
        return 0
    if a.cmd == "add":
        if a.exp not in specs:
            print(f"unknown experiment {a.exp}; have {sorted(specs)}", file=sys.stderr)
            return 2
        if not FROZEN.exists() or sha not in FROZEN.read_text():
            print("refusing: protocol.yaml is not frozen at its current hash - "
                  "run `trial_log.py freeze` (and commit) before logging trials",
                  file=sys.stderr)
            return 2
        values = dict(kv.split("=", 1) for kv in a.values)
        try:
            add_trial(a.exp, values, specs[a.exp], raw=a.raw, note=a.note, sha=sha)
        except ValueError as exc:
            print(exc, file=sys.stderr)
            return 2
        print(f"logged {a.exp} -> {trials_path(a.exp).relative_to(ROOT)}")
        return 0
    exps = a.exps or sorted(specs)
    text = "\n\n".join(render(e, specs[e], read_trials(e), sha) for e in exps)
    print(text)
    if a.md:
        a.md.write_text(f"# Evidence report — {dt.date.today()}\n\n{text}\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
