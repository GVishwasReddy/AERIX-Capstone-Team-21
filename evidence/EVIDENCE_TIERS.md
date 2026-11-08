# Evidence tiers: what is proven, and how

Rule from the 2026-10-08 evidence review: **nothing moves from SIM-TESTED to
ON HARDWARE until a log says so.** Update this file after every trial, with
the date and the log path.

| Tier | Means |
|---|---|
| **ON HARDWARE** | Ran on the Pi + Pixhawk (+ phone); a log or receipt on the aircraft records it |
| **BENCH-SW** | The real code, driven end to end by tests with only hardware edges stubbed |
| **SIM-TESTED** | Simulation through the real code (`sim/`, Monte Carlo); a model, not a flight |
| **OFFLINE** | Measured on recorded data, not live |
| **DESIGN** | Written and unit-tested at most; **keep out of claims and slides as a finished feature** |

## Claims

| Claim | Tier | Evidence | Gap / next step |
|---|---|---|---|
| Phone handshake → release → receipt chain | **ON HARDWARE** | Receipts 27 Aug – 23 Sep 2026 (19 valid); `logs/ble` 4–5 Oct (364 operations) | Counts bench releases and 3 ground releases from before the 27 Sep fix: proves the chain, **not 19 deliveries** |
| Single-use token spent only after release | **ON HARDWARE** | Spent markers on the Pi | Exp. 5 |
| Fails closed with a named reason (AUTH_FAIL, SIGNATURE_FAIL, GEOFENCE_FAIL, NOT_READY, PILOT_OVERRIDE) | ON HARDWARE (codes seen in logs) + **BENCH-SW** (17-row attack table, 11 mutations all caught) | `tests/test_attack_table.py` | Exp. 1 with a real phone |
| Session binding / nonce expiry (anti-replay) | **BENCH-SW** | `test_ble_session.py`, `test_attack_table.py` T1–T5 | Exp. 1 |
| Token not recoverable from a captured card | **BENCH-SW** (8 Oct) | `test_order_store.py` A/B on a directory | Exp. 2 on a real `dd` image |
| Signed, hash-chained, write-ahead release ledger | **BENCH-SW** (format new 8 Oct) | `test_receipt_ledger.py`, `test_ble_release_ledger.py` | First flight with it deployed |
| Signed RESULT to the phone | **BENCH-SW** (drone side only) | `test_attack_table.py` T18 | App must verify it |
| Formal security model | **MACHINE-CHECKED (bounded)**: Verifpal 1.6.4, 2 sessions, 2026-10-08. Today's app: 4/6 fail (T29). With the 16-byte nonce fix: every parcel-releasing query passes. v2: 7/7 pass. | `docs/security/verifpal_results/`, `scripts/verify_protocol.sh` | ProVerif (unbounded) on a Mac; ship the app fix |
| T29 signing-oracle attack | **BENCH-SW, attack confirmed** against the real drone code | `test_attack_table.py::test_T29` (strict xfail) | App item 0, then the xfail flips and must be turned into a passing test |
| Person detection at the drop point | **OFFLINE**: 64.4% recall, 57.4% precision, 40 aerial frames | — | Exp. 4: ≥ 90% recall, in flight, by person size |
| Person scan + precision drop at 1 m (TF02-Pro) | **SIM-TESTED**; deployed but unflown at 26 Sep | `test_person_scan.py`, `test_precision_drop.py` | Tethered, then exp. 5 |
| Picking the phone holder among 2–3 people | **SIM-TESTED** | `sim_phone_pick.py`: 0.17–1.33% wrong, 42–77% identified (600 trials/row, literature radio values) | Fit the radio model to `logs/rssi`; exp. 6 |
| LiDAR avoidance around the handover | SIM-TESTED + bench noise A/B (21 Sep) | `test_*avoidance*.py`, docs/PROJECT_NOTES.md §13 | Exp. 7 |
| 5 m ceiling, stand-down on pilot takeover | SIM-TESTED; takeover seen ON HARDWARE (27 Sep) | `test_transmitter_authority.py` | — |
| Novelty layer (`drone_stack/novelty/`: markerless landing zone, dual-factor auth, motion abort) | **DESIGN**. Off by default, never deployed, every threshold `# GUESSED` | `tests/novelty/` | Fly it and set thresholds from data, **or keep it out of the claims** |

## Records that disagreed (evidence review, item 8)

| Mismatch | Resolution |
|---|---|
| 19 receipts vs 21 used-token files | **Likely the two torn receipts.** `receipt_ledger.py` documents a 0-byte receipt (19 Sep) and a 299-zero-byte one (21 Sep) whose tokens were already spent: power lost after an unsynced write, before the servo fired. 19 + 2 = 21. **Confirm on the Pi:** `scripts/verify_receipts.py reconcile` should list exactly those two orders as spent-without-receipt. |
| docs/PROJECT_NOTES.md said SMART_RTL at 3 m; config says RTL at 5 m | Config is the truth: `delivery.return_mode: RTL`, `safety.max_altitude_m: 5.0` (operator, 24 Sep). docs/PROJECT_NOTES.md corrected 8 Oct. |
| Geofence 5 m vs 12 m | 12 m (`delivery.ble_geofence_m`). 5 m never passed (4 Oct: 5.3–6.7 m with the recipient under the camera). Pinned by `test_the_ble_geofence_has_one_value_everywhere`. |
| Test count quoted as static | Fresh run, dated, below. |

**Test suite, 2026-10-08 (cloud session, Python 3.13, branch `main-rmuplh`):
1149 passed, 1 skipped** on Python 3.13 and 3.11 (the skip needs `libcamera`). Baseline the same day,
before this work: 1080 passed. Not yet re-run on the Pi. Do not quote a test
count without a date.
