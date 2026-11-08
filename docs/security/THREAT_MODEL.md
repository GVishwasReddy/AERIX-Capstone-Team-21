# AERIX last-metre release — threat model

Written 2026-10-08 in answer to the evidence review ("no threat model and no
proof; SecAuthUAV leads here"). It covers the step neither base paper covers:
**releasing a parcel to the right person**, from the moment the drone holds
over the drop point to the moment the servo fires.

Every row names the test that pins it. "Mitigated" means a test drives the
real peripheral code through the attack and checks three things: the named
refusal code, a servo that never moved, and a token still unspent. It does
**not** mean flight-tested; the tier column says which.

```bash
pytest tests/test_attack_table.py tests/test_order_store.py tests/test_ble_session.py
```

## 1. What is protected

| Asset | Why it matters |
|---|---|
| The parcel (servo release) | The whole claim. A release to the wrong party is the worst outcome. |
| The order `deliveryToken` | 16 random bytes, issued per order by the order server. Holding it lets you play the phone's side of the handshake. |
| The delivery record | Ed25519-signed, hash-chained ledger (`receipt_ledger.py`). The proof that a release did or did not happen. |
| The recipient's position | Signed GPS fixes from the phone. Personal data. |

## 2. Who is trusted

| Party | Trusted for | Not trusted for |
|---|---|---|
| Order server (Firestore) | Issuing the token to the right phone and drone | Anything on the BLE link |
| Recipient's phone app | Holding the token, signing its own GPS | Its outcome display, unless it verifies `RESULT_SIGNED` |
| Drone (Pi + Pixhawk) | Gates, ledger, servo | Nothing beyond its own sensors |
| BLE link | Nothing. **Unencrypted, unpaired.** The attacker reads, writes and replays everything. | — |

The attacker is the standard Dolev-Yao network attacker on BLE, plus the
physical attacks in §4: a stolen SD card, a captured aircraft, a relay, and a
second phone in radio range.

## 3. Protocol attacks (BLE)

| ID | Attack | What stops it | Test | Status |
|---|---|---|---|---|
| T1 | Sniff a whole session and replay it later | Session bound to the central that read the nonce; unanswered nonce dies in 10 s, idle session in 30 s | `test_T1` + `test_ble_session.py` | Mitigated, bench |
| T2 | Replay DROP after the parcel has gone | Idempotent `_delivered` state; the token is spent once | `test_T2` | Mitigated, bench |
| T3 | Answer a stale nonce | `NONCE_TTL_S` = 10 s, 31× the slowest measured phone | `test_T3` | Mitigated, bench |
| T4 | Wrong or guessed token | HMAC-SHA256, constant-time compare | `test_T4` | Mitigated, bench |
| T5 | Second phone writes into the recipient's live session | Device binding: writes from a different central are dropped and leave the session untouched | `test_T5` | Mitigated, bench |
| T6 | Tamper with the phone's coordinates | Each fix is MAC'd over nonce‖fix | `test_T6` | Mitigated, bench |
| T7 | DROP signed for coordinates other than the verified ones | DROP MAC covers nonce‖fix‖"DROP" | `test_T7` | Mitigated, bench |
| T8 | **Relay** a genuine phone that is far away | The phone signs its *own* position, so the geofence fails. A relay of a phone that really is near cannot be told apart, but then the recipient is there. | `test_T8` | **Bounded, not prevented**, and only once T29 is fixed: today a relay can choose the position (T29). Exp. 3 measures the geofence against GPS noise. |
| T9 | GPS jump, e.g. a glitch or a fake fix spliced in | 5 consistent fixes within 12 s; any jump over 10 m restarts the window | `test_T9` | Mitigated, bench |
| T15 | Bystander reads RESULT or ACK | Both withheld from any central without the live authenticated session | `test_T15` | Mitigated, bench |
| T16 | Fake drone collects the phone's GPS and DROP | Mutual auth: ACK = HMAC(token, nonce‖"ACK") | `test_T16`, model Q3 | Mitigated, bench |
| T17 | Reuse one message's MAC as another (AUTH as a GPS signature, and so on) | **The coordinate parser.** All MACs share one key with no explicit labels. An AUTH MAC *does* verify as a GPS signature over the empty string; only `float("")` failing stops it. | `test_T17` | Mitigated by parsing. **Protocol v2: add explicit labels.** |
| T18 | Fake drone, or relay, tells the phone `RELEASED` while the parcel is still aboard | New `RESULT_SIGNED` characteristic (…567e): code ‖ 0x00 ‖ HMAC(token, nonce‖"RESULT"‖code)[:16] | `test_T18`, model Q5 | **Drone side shipped. The app must verify it**, and it is only unforgeable **once T29 is fixed**: today's phone will sign a RESULT tag for a fake drone if asked. |
| T22 | Fake drone replays an old (nonce, ACK) pair to be "trusted" | Nothing yet: the phone adds no nonce of its own. It gains nothing on the real drone (T1), but it can show a fake outcome unless T18 is checked. | model Q4 | **Open. Needs the app:** a phone nonce in AUTH. |
| T23 | Stranger reads NONCE to reset the recipient's session | Accepted: a nuisance on a par with jamming. The stranger cannot *use* the phone's nonce. | `test_ble_session.py` | Accepted (DoS only) |
| **T29** | **Fake drone uses the phone as a signing oracle.** The app signs *any* NONCE it reads (it only checks non-empty). Shown `realNonce‖pos` it returns a valid GPS signature for any position; shown `realNonce‖pos‖"DROP"` it returns a valid DROP MAC. With one device near the phone and one near the drone, the parcel releases while the recipient is far away. | **Nothing on the drone can tell**: the MACs are genuine. Fix in the app: refuse any nonce that isn't exactly 16 bytes (then every MAC input has a unique length). Protocol v2 removes it by design. | `test_T29` (strict xfail: the attack succeeds today), Verifpal `aerix_handshake.vp` (`gps_sig` FAIL) | **OPEN, CRITICAL. Found by the formal model, 2026-10-08.** App fix item 0 in `docs/app/APP_CHANGES_PROMPT.md`. |

## 4. Mission-state attacks: valid cryptography, wrong moment

| ID | Attack | What stops it | Test | Status |
|---|---|---|---|---|
| T10 | DROP while the aircraft is on the ground (the 27 Sep bug) | Two independent signals: the navigator's `on_ground` flag *or* disarmed telemetry. Only the bench token may release on the ground. | `test_T10`, `test_ble_ground_release.py` | Mitigated, bench. Root cause seen on hardware. |
| T11 | DROP during a pilot takeover | `PILOT_OVERRIDE`, token kept | `test_T11` | Mitigated, bench |
| T12 | DROP before a person is locked below | `NOT_READY` until the navigator opens the window | `test_T12` | Mitigated, bench |
| T13 | Valid token for a *different* order than the one being flown | Order-match gate | `test_T13` | Mitigated, bench |
| T14 | Release while the audit record cannot be written | Write-ahead `release_intent` must be fsynced first, else no release | `test_T14`, `test_ble_release_ledger.py` | Mitigated, bench |

## 5. Captured aircraft — where SecAuthUAV leads

| ID | Attack | What stops it | Test | Status |
|---|---|---|---|---|
| T19 | Pull and image the SD card | The live order file is in RAM (`/run/aerix-ble`) and its token is AES-256-GCM sealed under a key derived from the SoC serial, which is not on the card. Spent markers and receipts hold only SHA-256 commitments. | `test_order_store.py` (`old_layout_is_recoverable_new_layout_is_not`) | Mitigated in software. **Exp. 2 on a real card image** (`scripts/token_capture_check.py`) |
| T20 | Capture the whole aircraft powered off | RAM is gone at power-off, so no live token exists to steal | same | Mitigated in software |
| T21 | Root on the **running** aircraft | **Not mitigated.** The token is in the peripheral's memory while an order is live. Closing this needs the key in hardware the CPU cannot read: a secure element (e.g. ATECC608) or TPM, or a PUF as SecAuthUAV uses. | — | **Open. SecAuthUAV is stronger here; say so.** |

Note on T19: the SoC serial is a *binding*, not a secret. Anyone holding the
whole board can read it, which is why the file also lives in RAM.

## 6. Accepted or out of scope (stated so nobody claims otherwise)

| ID | Item | Position |
|---|---|---|
| T24 | BLE jamming | Fails closed: no handshake, so no release; the drone returns with the parcel. |
| T25 | Someone steals the recipient's **phone** and stands below | The token is a bearer credential. The camera confirms *a person*, not *the recipient*. Sensors 2022 matches faces; AERIX deliberately stores none. Mitigation belongs in the app: device unlock or biometric before DROP. |
| T26 | Spoof the *drone's* own GPS | Out of scope. The geofence is relative to the drone's fix; ArduPilot's EKF checks own that. |
| T27 | Compromise the order server (Firestore) | Out of scope. It is the root of trust that issues tokens. |
| T28 | The recipient spoofs their own phone's GPS | Low impact: it moves their own parcel. The person lock still requires someone below. |

## 7. Formal models: machine-checked, 2026-10-08

ProVerif could not be installed in the cloud session (upstream and package
mirrors blocked). **Verifpal 1.6.4** was built from source there and run on
four models. Raw output, dated: `docs/security/verifpal_results/`. Re-run:
`scripts/verify_protocol.sh`.

| Model | What it is | Result |
|---|---|---|
| `aerix_handshake.vp` | today's protocol, today's app | **4 of 6 fail.** `gps_sig` (T29 signing oracle), `ack` and `result_tag` (replay to the phone), `freshness nonce`. Token secrecy and `drop` pass, but `drop` passes only because the model's structured CONCAT hides the byte-level trick in T29; on real bytes it fails too (`test_T29`). |
| `aerix_handshake_v1_1.vp` | + the app refuses non-16-byte nonces | **3 of 6 fail, all phone-side replay** (T22). Token secrecy, `drop` and `gps_sig` **pass**: no attack found that releases a parcel. |
| `aerix_handshake_v2.vp` | proposal: labels on every MAC + a phone nonce | **All 7 queries pass**, including freshness of both nonces. |
| `aerix_handshake_plain_result.vp` | control: the phone trusts plain RESULT | fails as it must (forged / replayed outcome), so the tool does find these attacks |

What these results are, and are not:

- **Bounded.** Verifpal reports "search exhausted at 2 sessions": no attack
  exists with up to two parallel runs of the protocol. It is not an
  unbounded proof.
- **Sound but incomplete,** in the tool's own words: every attack it reports
  is real, but it may miss attacks.
- **Abstract.** It uses ideal MACs, no truncation, and CONCAT is structured,
  not bytes. The `.vp` headers list every abstraction. Byte-level confusion
  is covered by `test_T17` and `test_T29` instead.
- **Physical properties are outside the model.** Geofence, person lock and
  relay distance are tested as T8–T13.

`aerix_handshake.pv` (ProVerif, unbounded sessions) is unchanged and still
**not run**. On a Mac: `opam install proverif`, then
`PROVERIF=1 scripts/verify_protocol.sh`. Its hand-derived Q2 "true" is wrong
for today's app: Verifpal found T29. Expect ProVerif to agree once its
model is given the untagged AUTH.

## 8. Do the tests actually test the gates? (2026-10-08)

Each gate below was disabled in `drone_ble_peripheral.py` and the attack
table re-run. Every mutation turned it red, then the file was restored.

| Gate removed | Result |
|---|---|
| Device binding on DROP | red |
| Geofence always passes | red |
| DROP MAC unchecked | red |
| Ground-release interlock | red |
| Pilot-override refusal | red |
| Release without a durable ledger intent | red |
| Nonce never expires | red |
| GPS jump check | red |
| Person-lock gate | red (2 tests) |
| Order-match gate | red |
| Delivered-idempotency | red |

## 9. Protocol v2: needs the app, which is not in this repository

0. **Now, app only:** refuse any nonce that isn't 16 bytes (T29). Verified: closes every parcel-releasing attack (v1.1).
1. Explicit labels on every MAC: `"AERIX/v2/AUTH"`, `/GPS`, `/DROP`, `/RESULT` (T17). Verified with the phone nonce: all 7 queries pass (v2).
2. A phone nonce in AUTH, so the ACK is fresh for the phone too (T22, Q4).
3. Verify `RESULT_SIGNED` before showing "delivered" or writing the phone's receipt (T18).
4. Device unlock or biometric before DROP (T25).
5. Put the app source under version control next to this file (evidence review, item 4).
