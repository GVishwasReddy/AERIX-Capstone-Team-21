# Pre-flight checklist

For every flight that leaves the ground, including evidence-plan experiments
3–7. The bench (props off) needs only section A.

Current state, 2026-10-08: **NO-GO for flight.** `safety.failsafes_enabled`
is `false` in `config/real.yaml`, left off on the operator's instruction for
bench work. `scripts/preflight_check.py` will keep saying NO-GO until that
changes. That is intended.

## A. Software (on the Pi, GCS stopped for `--fc`)

```bash
sudo systemctl stop aerix-gcs.service
scripts/preflight_check.py --fc          # must end in GO
sudo systemctl start aerix-gcs.service
```

It checks, read-only:

| Blocker (NO-GO) | Warning |
|---|---|
| `safety.failsafes_enabled: false` | servo travel still at full 500–2500 µs (payload and aux) |
| `delivery.hover_mode` is POSHOLD / LOITER / ALT_HOLD (the 27 Aug crash) | any altitude above the ceiling (silently clamped) |
| `delivery.auto_accept: true` | 4S battery thresholds on a 6S pack |
| FC `ARMING_CHECK` ≠ 1 | no FC-side fence (`FENCE_ENABLE` 0) |
| FC `FS_THR_ENABLE` / `FS_GCS_ENABLE` = 0 | `PILOT_SPEED_DN` 0 (= 2.5 m/s descent) |
| FC `BATT_FS_LOW_ACT` / `BATT_FS_CRT_ACT` = 0 | STABILIZE / ACRO on any mode-switch position |
| | `RTL_ALT` above the ceiling |

## B. Transmitter (needs the operator; cannot be checked from the Pi)

- [ ] Mode-switch positions: no STABILIZE or ACRO (docs/PROJECT_NOTES.md §11, outstanding). Use LOITER / RTL / LAND.
- [ ] Throttle stick at **mid** before handing over to autonomy.
- [ ] RC8 motor E-stop and RC6 LAND both tested on the ground today (watch for the STATUSTEXT toast).
- [ ] Decide the E-stop behaviour in the air and brief everyone present: it cuts the motors, so it is a fall.

## C. Aircraft

- [ ] 6S pack charged; voltage on the GCS above `battery_low_voltage` + 1 V.
- [ ] Props tight, correct rotation; LiDAR and the downward TF02-Pro unobstructed.
- [ ] Payload servo locks at `payload.lock_us` with a parcel loaded. Pull test by hand.
- [ ] 3D fix, ≥ `min_satellites`; home latched on the GCS map at the launch point.

## D. Site and people

- [ ] Open area, nobody under the flight path except a briefed recipient.
- [ ] First flights of any new feature are **tethered** (evidence review, item 1).
- [ ] Recorder shows ARMED → recording (docs/PROJECT_NOTES.md §12).

## E. Evidence (after every flight)

- [ ] `scripts/trial_log.py add <exp> …` with `--raw` pointing at the log.
- [ ] Copy `logs/ble/`, `logs/rssi/`, the dataflash log and the recording off the Pi.
- [ ] `scripts/verify_receipts.py` and note the receipt count with its date.
