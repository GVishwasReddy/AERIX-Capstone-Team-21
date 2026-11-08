# From "simulation only" to a filed patent: the person-picking claim

The claim (`CLAIM_CHART.md` §0): with 2+ people under the drone, **fly to
chosen vantage points to tell them apart by the phone's RSSI, and decline
unless the pick survives leave-one-out and one more look.**

## Where it stands (2026-10-08)

| Piece | State |
|---|---|
| Algorithm | **Implemented and wired on the real data path**: BLE `RssiSampler` (HCI_Read_RSSI) → `ble_phone_signal` → `PhoneLocator` → `probe_planner.next_vantage` → navigator accept/decline. 100+ unit tests. |
| Wrong-pick rate | **Simulation only**: 0.17–1.33% wrong, 42–77% identified, with literature radio values |
| Radio model of *this* aircraft | **Not measured yet.** `scripts/fit_radio_model.py` now does it from `logs/rssi`. |
| Flight decision record | **Now automatic**: `logs/rssi/picks-*.csv`, one row per pick/decline |
| Public disclosure | Elements of the *old* claim are public (jalanths repo, 1 Oct); this claim's core is not |

## The steps, in order

| # | Step | Who | Time | Output |
|---|---|---|---|---|
| 1 | Make the jalanths repo private; publish nothing more about the method | team | today | stops further disclosure |
| 2 | Meet the university IP office with `CLAIM_CHART.md` + this file; ask for a **provisional** on §0 claim 1 | team + IP office | this week | filing date fixed (India: provisional specification; US: provisional application) |
| 3 | Ship the app's 16-byte nonce check (T29) | app developer | 1 day | the handshake the claim relies on is sound |
| 4 | Deploy (`scripts/deploy_to_pi.sh --apply`), freeze the protocol | team | 1 hour | |
| 5 | **Exp. 8: radio calibration**: one phone at a marked spot, orbit + passes to 12 m | 2 people | 10 min of flying | `radio_fit_*.json`, measured `locator_*` values |
| 6 | `sim_phone_pick.py --radio <fit>`: wrong-pick rate on **your** radio; put the fit in `real.yaml` | anyone | 10 min | simulation now anchored to measurement |
| 7 | **Exp. 6: 2–3 people, one phone**, ≥ 20 runs per group; fill `truth_idx` in `picks-*.csv` | 4 people | 1–2 field sessions | the claim's own flight evidence |
| 8 | Compare exp. 6 with step 6's prediction | anyone | 1 hour | agreement makes the simulation credible for cases you did not fly |
| 9 | Professional prior-art search on claim 1 (CPC B64U 2101/60, G01S 5/02, H04W 4/02) | IP office / agent | weeks | go / narrow / no-go |
| 10 | Complete specification within 12 months of the provisional, with steps 5–8 as the worked example | agent | | full application |

Steps 1–2 do not wait for steps 3–8. A provisional fixes the date for what
it describes; the flight data strengthens the complete specification later.

## What exp. 6 must show to support the claim

From `evidence/protocol.yaml`: **wrong pick ≤ 5%** with at least 20 runs per
group. Declines are allowed and expected: the claim is that it refuses
rather than guesses.

Report it honestly, with the Wilson interval: 0 wrong out of 20 bounds the
true rate below about 16%, not at 0%. More runs narrow it, so 40–60 runs
per group is the better target if the field days allow.

## If the numbers disappoint

- **Too many declines** (it rarely names anyone): widen the probe budget
  (`ambiguity_probe_budget_s`) or allow vantage points further out
  (`locator_max_offset_m`). The claim is about *not guessing*, so declining
  is not a failure of the claim.
- **Any wrong picks**: check that the fitted radio is in `real.yaml` (step 6).
  Then look at `loo_min` in `picks-*.csv`; a wrong pick with a high `loo_min`
  is a radio-model problem, not a threshold one.
- **The phone's own GPS dominates the decision**: that is what the
  leave-one-out check exists to catch. Compare the GPS-excluded probability
  in the logs.
