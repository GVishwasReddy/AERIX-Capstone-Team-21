# AERIX claim chart (draft for the university IP office)

**Not legal advice.** This is an engineering draft to hand to a patent attorney
or your university's IP office. They decide what is claimable, where, and when.

Status, 2026-10-08:

- **⚠️ PUBLIC DISCLOSURE FOUND — take this to the IP office first.**
  `github.com/jalanths/aerix-drone-delivery` is **public**, first commit
  **2026-10-01**. It holds the Flutter app, a copy of `drone_stack/` and the
  BLE peripheral, and a README that describes the method. It discloses
  claim elements **[a] [b] [c] [f]** below and a basic RSSI person pick
  (`navigation_node._rssi_pick`, `phone_locator.score_candidates`). It does
  **not** contain [d] (vantage planning, decline unless robust and
  confirmed), [e] (signed write-ahead record), session binding, the
  RAM-sealed token or RESULT_SIGNED. Those were written after 1 Oct, in this
  private repo.

  What that means, roughly (the IP office decides):
  - **India and Europe** have no general grace period for your own online
    disclosure, so [a][b][c][f] are likely prior art against you there.
  - **The US** allows a 1-year grace period for the inventors' own
    disclosure (35 U.S.C. 102(b)(1)(A)), until about **2027-10-01**.
  - Making the repo private now does not undo the disclosure, but it stops
    it from spreading further. Ask the repo's owner about that today.
  - **File a provisional application quickly** for what is still
    undisclosed ([d], [e] and the rest), so its date is fixed before
    anything else is published.
- **Search depth.** Two base papers plus a structured web pre-search (§5).
  The patent texts below were identified from search results and summaries.
  **Their full claims have not been read**, and every "disclosed?" cell must
  be checked against the full text before this chart is relied on.
- **Search depth.** Two base papers plus a short web search. The patent texts
  below were identified from search results and summaries. **Their full
  claims have not been read**, and every "disclosed?" cell must be checked
  against the full text before this chart is relied on. Assume the field is
  crowded until a professional search says otherwise.

## 0. Revised strategy after the disclosure (2026-10-08, supersedes §1 as claim 1)

With [a][b][c][f] public since 1 Oct, the independent claim has to rest on
what is not public. The strongest candidate is [d]. It is also the element
the pre-search (§5) found least covered: the art links phones to people by
RSSI (US10083558 family), but nothing found **chooses where to fly to
separate the candidates** and **declines unless the pick is robust and
confirmed**.

> **Claim 1 (draft).** A method of handing a payload from an unmanned aerial
> vehicle (UAV) to the holder of a recipient device when a camera on the UAV
> detects a plurality of persons at a drop point, comprising:
>
> (i) maintaining, for each detected person, a posterior probability that
> the person holds the recipient device, from received signal strengths of
> a short-range wireless link with the device measured at recorded UAV
> positions, under a model in which the device's reference power is unknown
> and common to all measurements;
>
> (ii) repeatedly selecting a next measurement position as the reachable
> position that most reduces an upper bound on the probability of confusing
> any two persons, per unit of flight time, moving the UAV there and
> measuring again;
>
> (iii) designating a person only if (a) their posterior is at least a first
> threshold, (b) they remain the most probable person, with posterior at
> least a second threshold, when any one group of measurements, or the
> device's self-reported position, is excluded, and (c) they are designated
> again after a further measurement from an additional position; and
>
> (iv) releasing the payload toward the designated person, or, when no
> person is designated within a time budget, **returning with the payload**.

Dependent claims would then add the publicly known gates as context
(authenticated session, signed position against the UAV's own fix, person
lock), plus:

- D1: the Bhattacharyya / Mahalanobis form of the bound in (ii) (`probe_planner.py`).
- D2: Student-t residuals and a GPS-outlier mixture (`phone_locator.py`).
- D3: the write-ahead record ([e]).
- D4: the RAM-sealed credential.
- D5: the signed outcome.

**Candidate second independent claim [e]** (weaker: tamper-evident
hash-chained logs are well known, e.g. Crosby & Wallach, USENIX Sec 2009):
before actuating a payload release, durably commit a signed, hash-chained
release-intent record; spend the credential only after the release is
confirmed; at restart, close any intent without an outcome as "release
unconfirmed". It is novel as an *ordering around an actuator* at best. Let
the attorney judge obviousness.

**Evidence status of claim 1: SIMULATION ONLY** (600 trials/row). An
examiner will not require flight data, but your own credibility, and
enablement of the thresholds, benefit from experiment 6 flown before or
soon after a provisional.

## 1. Draft independent claim (method) — as first drafted

*(Kept for reference. [a][b][c][f] are now public; see §0.)*

The narrow, strong choice is the evidence review's: claim the **handover
method**, not the whole stack.

> A method of releasing a payload from an unmanned aerial vehicle (UAV) to a
> recipient, comprising:
>
> **[a]** holding the UAV at a drop point and detecting, with a camera on the
> UAV, one or more persons below, and opening a release window only while a
> detected person is locked;
>
> **[b]** establishing a short-range wireless session with a recipient device
> using a challenge issued by the UAV and a response computed from a
> single-use order credential, the session being bound to that device and
> expiring when unanswered or idle;
>
> **[c]** receiving from the recipient device a sequence of position reports,
> each authenticated with the order credential and the session challenge,
> and accepting the recipient's position only when a plurality of
> consecutive reports agree within a jump bound and their median lies within
> a radius of the UAV's **own** satellite position;
>
> **[d]** when two or more persons are detected, determining which person
> holds the recipient device from the received signal strength of the
> session, measured from a sequence of vantage points each selected to
> maximise the expected separation of the candidates' predicted signal
> strengths, and **declining the delivery** unless one candidate exceeds a
> posterior threshold, remains the most probable when any single item of
> evidence is removed, and is confirmed from a further vantage point;
>
> **[e]** upon a release command authenticated with the credential, the
> challenge and the accepted position, and only while the window of [a] is
> open and the UAV is airborne under autonomous control, durably recording a
> signed release-intent record **before** actuating the release; and
>
> **[f]** marking the order credential as spent only after the physical
> release is confirmed, and otherwise retaining it and reporting a named
> refusal reason to the recipient device.

Candidate dependent claims:

| # | Adds | Code |
|---|---|---|
| D1 | [d] vantage selection by minimising a pairwise Bhattacharyya confusion bound per second of flight, with the phone's unknown reference level marginalised (only contrasts between vantages count) | `nodes/probe_planner.py` |
| D2 | [d] heavy-tailed (Student-t) RSSI residuals and a GPS-outlier mixture in the posterior | `nodes/phone_locator.py` |
| D3 | [e] records hash-chained and signed with the UAV's own key; a dangling intent is closed as "release unconfirmed" at next start | `ble_handshake/receipt_ledger.py` |
| D4 | the credential held only in volatile memory and sealed to a hardware identifier of the UAV | `ble_handshake/order_store.py` |
| D5 | the outcome returned to the device authenticated under the session challenge | `RESULT_SIGNED`, `drone_ble_peripheral.py` |
| D6 | release refused on the ground for a real order, by two independent signals (navigator flag, disarmed telemetry) | `_request_drop_worker` |

## 2. Element-by-element comparison

Y = disclosed, P = partly, N = not disclosed, ? = not determinable from what
was read. Patent rows are based on **summaries only** (see Status above).

| Element | SecAuthUAV (TVT 2020) | Sensors 2022 (face ID) | Qualcomm US9359074B2 (pub. US20160068264) | Amazon US10,198,955 | US9313399 (people in image via device signals) | US12300033B2 (Bosch / SUNY) |
|---|---|---|---|---|---|---|
| [a] camera person-lock gates a release window | N (no delivery) | P: face recognised; no secure release step | ? (images for delivery *confirmation*, per claim 3 summary) | N: markers / landing zone | N | P: person detection with camera + wireless; no UAV release |
| [b] challenge-response with a single-use order credential, session-bound | P: PUF mutual auth, UAV–GS and UAV–UAV only | N | **P/Y: one-time purchase code authenticated before landing** | N | N | N |
| [c] signed recipient position vs the UAV's own GPS, temporal consistency | N | N | ? | N | N | N |
| [d] RSSI vantage planning to separate candidates; decline unless robust and confirmed | N | N | N | N | P: wireless signals tag persons in an image (no planning, no decline) | P: wireless + image identification (no planning, no decline) |
| [e] signed write-ahead intent before actuation | N | N | N | N | N | N |
| [f] credential spent only after confirmed physical release; named refusals | N | P: alarm after 10 s | P: abort landing on failed authentication | N | N | N |

**Reading of the chart** (to be checked by the attorney):

- **Qualcomm is the closest art for [b] and [f].** A one-time code that
  gates landing and release is already there. Do not rely on "single-use
  token" alone. What it does not appear to show is binding the release to a
  *person lock* [a], a *signed position against the drone's own fix* [c],
  *declining among several people* [d], or *write-ahead recording before
  the actuator* [e].
- **[d] is the most distinctive element**, and the one with the least
  hardware evidence (simulation only). Wireless-plus-camera person
  identification exists (US9313399, US12300033B2). Choosing *where to fly*
  to tell candidates apart, and *refusing to guess*, does not appear in what
  was read. Prior art steers toward the strongest signal.
- **The combination [a]+[c]+[e]+[f]** is the defensible core with hardware
  evidence behind it (the release chain has run on the Pi and Pixhawk).

### References the 2026-10-08 desktop notes named but could not be confirmed

| Named | What the notes said | Status |
|---|---|---|
| "Amazon (RSSI-based location checks)" | RSSI from the recipient's device to confirm location | **Not found.** The confirmed Amazon art is US10,198,955 (markers / landing-zone verification) and the "Bring It To Me" device-location concept. Find the number or drop the row. |
| "EyeFi (camera plus Wi-Fi matching)" | camera image matched to a Wi-Fi device | **No Eye-Fi-assigned patent found.** US9313399 and US12300033B2 cover the idea; use those unless the attorney finds the Eye-Fi one. |
| "hold-the-package patents" | parcel held until confirmation | Unspecified. Ask the search professional. |

## 3. Evidence behind each element (see `evidence/EVIDENCE_TIERS.md`)

| Element | Tier today | What moves it up |
|---|---|---|
| [a] person lock gates the window | SIM + unit tests; person detection 64.4% recall offline | Exp. 4 (≥ 90% recall at 4 m in flight) |
| [b] session-bound challenge-response | **ON HARDWARE** (bench handshakes, `logs/ble`, 4–5 Oct) + attack table in software | Exp. 1 with a real phone |
| [c] signed position vs drone GPS | ON HARDWARE (geofence failures at 5 m logged 4 Oct led to 12 m) | Exp. 3 (accept-rate curve) |
| [d] vantage planning + decline | **SIM only** (600 trials/row) | Exp. 6 |
| [e] signed write-ahead intent | software + tests; ledger format is new (8 Oct) | Any flight with the new ledger |
| [f] spent only after release | **ON HARDWARE** (receipts 27 Aug – 23 Sep) | Exp. 5 |

## 4. Before filing

1. A professional prior-art search (patents + papers), including the full
   claims of US9359074B2, US9313399 and US12300033B2.
2. Experiments 4, 5 and 6 flown and logged (`evidence/`): the last metre
   is the claim, and it has not been flown end to end.
3. Run the formal model (`docs/security/aerix_handshake.pv`) and quote its
   real output.
4. Decide the jurisdiction with the IP office, and who the inventors are
   (who conceived each element, not who typed it). Keep the dated commits
   as the record.

## 5. Pre-search results (2026-10-08, web; not a professional search)

Searched per claim element. Each "closest" is from summaries; read the full
claims before relying on any row.

| Element | Closest found | What it has | What it lacks (vs AERIX) |
|---|---|---|---|
| Phone ↔ person by RSSI | **US10083558B1**, US11337063, US12273713 (security/public-safety app family) | ties a device to a person "by combining RSSI of the device and aspect ratio of the person"; drones track individuals by their phone's signal | no choice of vantage points to separate candidates; no decline rule |
| Drone hold position from BLE RSSI | US10477602, US10880716 | compares the RSSI of a Bluetooth signal across the drone's antennas to keep station relative to one phone | single known target, no candidate selection |
| Next measurement point for RF localisation | Dehghan et al., CRLB path planning for DRSSI localisation by UAVs; Kreucher et al., information-based sensor management (Signal Processing 2005) | choose the next UAV position to cut localisation uncertainty | continuous localisation (CRLB), not a discrete "which of N people" test with a common unknown reference power; no release gating |
| Decline and leave if not the right recipient | **US9646283** (secure payload deliveries via UAVs) | instructs the UAV not to hand over the payload and to leave when the person is not verified | trigger is failed verification, not ambiguity among several people |
| Identify the customer by camera | US10139817 | facial recognition against a database | biometric enrolment, which AERIX avoids |
| One-time code gates landing / release | **US9359074B2** (Qualcomm) | one-time purchase code authenticated before landing; release and confirmation | — (covers [b]/[f] in part) |
| Authenticate before release | US11847604, US12165097 (medication UAV); US11132635, US10026054 (receiving apparatus) | authentication step before release | method unclear from summaries |
| Phone as the location source | US20180049575A1 | customer's smartphone fixes the drop point | no check of phone against drone position before release |
| Geofence before release | US10592843 | beacons on drone and box form a geofence; Bluetooth between them | box, not a person's phone |
| Operator checks the customer by camera | US9536216B1, US20210229811 | a human decides from the video | not automatic |
| Recipient authentication (paper) | PLOS ONE 2021, "decentralized hybrid computing consumer authentication framework" | OTP + face recognition | no BLE, no signed position |
| **Your own public repo** | **jalanths/aerix-drone-delivery (public since 2026-10-01)** | [a][b][c][f], basic RSSI pick, precision drop | [d], [e], session binding, sealing, RESULT_SIGNED |

Nothing found combines (i)–(iv) of claim 1. That is a reason to file, not
proof of novelty; a professional search covers CPC B64U 2101/60, B64D 1/12,
G01S 5/02, H04W 4/02, H04W 12/06 and non-English art.

## Sources consulted (2026-10-08)

- Qualcomm US9359074B2 / US20160068264A1: https://patents.google.com/patent/US20160068264A1/en, https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9359074
- Amazon US10,198,955: https://www.freepatentsonline.com/10198955.html
- US9313399: https://patents.google.com/patent/US9313399
- US12300033B2: https://patents.google.com/patent/US12300033B2/en
- SecAuthUAV (Alladi et al., IEEE TVT 2020, DOI 10.1109/TVT.2020.3033060): https://quspace.qu.edu.qa/handle/10576/36396
- Own public disclosure: https://github.com/jalanths/aerix-drone-delivery (first commit 2026-10-01)
- US10083558B1: https://patents.google.com/patent/US10083558
- US10477602 / US10880716: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10477602
- US9646283: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9646283
- US10139817: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10139817
- US11847604: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11847604
- US11132635: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11132635
- US20180049575A1: https://patents.google.com/patent/US20180049575A1/en
- US10592843: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10592843
- US9536216B1: https://patents.google.com/patent/US9536216B1/en
- PLOS ONE 2021 consumer authentication: https://journals.plos.org/plosone/article?id=10.1371%2Fjournal.pone.0250737
- CRLB UAV path planning (Dehghan et al.): https://www.researchgate.net/profile/SMM_Dehghan/publication/267633771_Optimal_path_planning_for_DRSSI_based_localization_of_an_RF_source_by_multiple_UAVs/links/54967aeb0cf2ec13375b3217.pdf
- Crosby & Wallach, tamper-evident logging: https://static.usenix.org/event/sec09/tech/full_papers/crosby.pdf
