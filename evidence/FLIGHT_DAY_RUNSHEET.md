# Flight-day run sheet: experiments 8, 5, 4, 6

What the claim needs flown. The pass lines are in `protocol.yaml`; this sheet
is how to collect the rows in one or two field sessions.

## Before the day

- [ ] App updated with **item 0** of `docs/app/APP_CHANGES_PROMPT.md`
  (16-byte nonce check). Without it the handshake has a known hole (T29), and
  an examiner reading your threat model will see it.
- [ ] Deploy with `scripts/deploy_to_pi.sh --apply`. The Pi test count must
  pass.
- [ ] Review the pass lines, run `scripts/trial_log.py freeze`, and commit
  `PREREGISTERED.txt`.
- [ ] Failsafes **on** for these flights (`safety.failsafes_enabled: true`),
  and `scripts/preflight_check.py --fc` says GO. Turn them back off after, if
  you still want bench state.
- [ ] Mode switch has no STABILIZE position; E-stop briefed; first runs tethered.

## Roles (4 people is comfortable)

| Role | Does |
|---|---|
| Pilot | sticks, E-stop, nothing else |
| GCS operator | ACCEPT & FLY, watches the panel, calls the ending |
| Recipient(s) | stand where told, hold the phone, do not walk under the drone |
| Logger | writes each row straight away (commands below), photographs the setup |

## Experiment 8: radio calibration (first, ~10 minutes, no handover)

The person-picking claim currently rests on a simulation using textbook radio
values. This flight replaces them with your aircraft's.

1. One person holds the phone (app open, authenticated) at a **tape-measured
   spot**. Write down its east/north metres from the launch point. Nobody
   else nearby should have Bluetooth on.
2. Fly the scan orbit at 4 m, then slow passes at 2, 5, 8 and 12 m from the spot.
   The aircraft's height compresses the distance range: passes only out to
   8 m gave a 2.7x span in simulation, too narrow to pin the path-loss slope.
   The fitter warns below 3x.
3. Fit and log:

```bash
scripts/fit_radio_model.py logs/rssi/rssi-$(date +%Y%m%d).csv --phone-en 3.2 18.7 \
    --out evidence/radio_fit_$(date +%Y%m%d).json
scripts/trial_log.py add exp8 session=$(date +%F) samples=<n> distance_ratio=<x> \
    path_loss_n=<n> ref_dbm=<A> rssi_sigma_db=<s> shadow_sigma_db=<s> \
    --raw evidence/radio_fit_$(date +%Y%m%d).json
scripts/sim_phone_pick.py --radio evidence/radio_fit_$(date +%Y%m%d).json   # re-measured wrong-pick rate
```

4. If you agree with the fit, copy the printed `locator_*` lines into
   `config/real.yaml` **before** experiment 6, so the aircraft flies the
   model that was just measured.

## Experiment 5: full handover, 10 runs per ending

Order: tethered over grass → untethered, one recipient, open field.

| Ending | Set-up | Correct ending |
|---|---|---|
| `no_person` | nobody at the drop point | drone searches 25 s, returns **with** the parcel |
| `lock_no_handshake` | person below, app closed | lock, 30 s window, returns **with** the parcel |
| `full_handshake` | person below, app open, phone verified | release at ~1 m AGL, then return |

After each run:

```bash
scripts/trial_log.py add exp5 ending=full_handshake correct_ending=yes released=yes \
  lock_to_release_s=14.2 release_agl_m=1.03 parcel_to_person_m=1.8 \
  --raw "logs/ble/ble-$(date +%Y%m%d).log" --note "run 3, wind light"
```

`release_agl_m` comes from the TF02 line in the GCS log;
`parcel_to_person_m` comes from a tape measure. On the two no-release
endings, leave the three measurements empty (`lock_to_release_s=`).

**Abort the series** if a parcel releases on a no-release ending, or the
drone descends below 1 m without a release. Log the row first, then stop.

## Experiment 4: person detection at 4 m, at least 30 passes

Can share flights with exp. 5.

- Vary the people (at least 5), their clothing (dark and light), and the
  light (morning and afternoon).
- One row per pass and size band. The size bands are the person's box height
  in the frame: small < 80 px, medium 80–160 px, large > 160 px.
- `people_detected` is counted from the recording; replay it on the GCS
  REPLAY button.

```bash
scripts/trial_log.py add exp4 person_size=medium people_present=1 people_detected=1 \
  false_lock=no --raw recordings/latest.mp4 --note "pass 7, grey hoodie"
```

## Experiment 6: two or three people, one phone, at least 20 per group

- People 3 m apart under the drop point, only one with the app open. Rotate
  who holds the phone.
- `pick` is `right`, `wrong` or `declined`.
- A decline is a correct, safe outcome; a wrong pick is the failure.

The aircraft now writes every pick and decline itself, to
`logs/rssi/picks-YYYYMMDD.csv`. Each row holds the outcome, every person's
position, the top probability, the worst leave-one-out probability, and
whether the probe and confirming look ran. After each run, fill in
`truth_idx`, the index of the person who really held the phone (people are
listed in `people_enu` order), then log:

```bash
scripts/trial_log.py add exp6 people=2 pick=right --raw logs/rssi/picks-$(date +%Y%m%d).csv
```

Keep every `logs/rssi/*.csv`. They also feed the radio-model fit, which
replaces the literature values `sim_phone_pick.py` uses today.

## After the day

```bash
scripts/trial_log.py report --md evidence/REPORT.md
```

Then move each claim's tier in `EVIDENCE_TIERS.md` only where a row says so,
copy the raw logs off the Pi, and commit everything with the date.
