# Evidence

What turns AERIX's claims from design text into something an examiner can
check. The 2026-10-08 evidence review put it plainly: *a table of dates,
counts and pass or fail, with the raw files behind it, is worth more than any
amount of design text.*

| File | What it is |
|---|---|
| `protocol.yaml` | The seven experiments and their **pass lines**, the one place they live |
| `PREREGISTERED.txt` | When each version of `protocol.yaml` was frozen (hash + commit). Created by `freeze`. |
| `trials/<exp>.csv` | One row per trial: date, code commit, operator, protocol hash, raw-log path, measurements |
| `EVIDENCE_TIERS.md` | Every claim with its tier: ON HARDWARE / BENCH / SIM-TESTED / DESIGN |

## Before the first trial (needs you, not code)

1. Read the pass lines in `protocol.yaml`. They are the review's *proposals*
   (90% recall, 10/10 handovers, ≤ 5% wrong pick, ≥ 1.5 m from a pole…).
   Change any you disagree with **now**.
2. `scripts/trial_log.py freeze` and commit `PREREGISTERED.txt`.
   `add` refuses to log until the current protocol is frozen.
3. From then on, a changed pass line is a new protocol version. The report
   marks every table where the line moved after data came in.

## Order (from the review: bench first, then flight)

| # | Experiment | Where | Needs |
|---|---|---|---|
| 1 | Attack table with a real phone | bench, props off | phone, Pi, Pixhawk |
| 2 | Token capture from a card image | bench | card reader; `scripts/token_capture_check.py` |
| 3 | Geofence curve, 0–18 m | open sky, drone on a tripod is fine | tape measure, phone |
| 4 | Person detection at 4 m, ≥ 30 passes | flight | volunteers |
| 5 | Full handover, 3 endings × 10 | tethered, then open field | one recipient |
| 6 | 2–3 people, one phone | flight | 2–3 volunteers |
| 7 | Pole avoidance, 10 passes | flight, pilot on the sticks | 0.3 m pole |

Experiment 1 is already covered **in software**: `tests/test_attack_table.py`
drives every row of `docs/security/THREAT_MODEL.md` through the real
peripheral. The bench run with a real phone makes it hardware evidence.

## Logging and reporting

```bash
scripts/trial_log.py add exp3 distance_m=6 accepted=yes reported_m=5.4 \
    --raw logs/ble/ble-20261012.log --note "open field, 14:05"
scripts/trial_log.py report                      # all seven, PASS/FAIL/NEED MORE
scripts/trial_log.py report --md evidence/REPORT.md
```

Every rate is reported with a 95% Wilson interval. 10/10 still only bounds
the true rate above about 72%, so write "10 of 10", never "100%".

Keep every raw log the `--raw` column points at. A row without its raw file
is a claim, not evidence.

## Safety, every flight

Failsafes are **off** in `config/real.yaml` for bench work, as asked. Before
any experiment that leaves the ground, run `scripts/preflight_check.py`.
It reports exactly what is still in bench state. See
`docs/PREFLIGHT_CHECKLIST.md`.
