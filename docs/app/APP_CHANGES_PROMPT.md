# Prompt for the phone-app changes (Flutter)

Hand everything below the line to whoever
works on the app), opened in the **Flutter app's own repository**. It is
self-contained: every byte format comes from
`drone_stack/ble_handshake/drone_ble_peripheral.py`, and the test vectors are
pinned on the drone side by `tests/test_protocol_vectors.py`.

---

## Task: harden the AERIX delivery app's BLE handshake (protocol v1, no wire changes)

You are working on the Flutter app the parcel recipient uses to unlock a
drone delivery over BLE. The drone (Raspberry Pi, BlueZ peripheral) was
hardened on 2026-10-08. The app must now (1) verify the drone's new signed
outcome, (2) follow the session rules exactly, (3) require the phone owner
before DROP, and (4) be testable and kept in version control.

**Do NOT change any existing MAC format, characteristic or byte layout.**
The drone accepts exactly the bytes below; any change breaks delivery.
Protocol v2 (explicit labels, phone nonce) needs a coordinated drone change
and is out of scope (see "Not now").

First, read the app's existing BLE code and tell me where each step below
lives. Then make the changes, one commit per numbered item, with tests.

### The protocol as the drone implements it (v1)

Service `12345678-1234-5678-1234-567812345678`, advertised as
"Target Drone Detected".

| Char | UUID suffix | Op | Bytes |
|---|---|---|---|
| NONCE | `...5679` | read | 16 random bytes; a read starts a NEW session for this connection |
| AUTH | `...567a` | write | `HMAC-SHA256(token, nonce)[:20]` |
| AUTH (ACK) | `...567a` | read | `HMAC-SHA256(token, nonce ‖ "ACK")[:20]`; 20 zero bytes if not authenticated |
| GPS | `...567c` | write | `s ‖ 0x00 ‖ HMAC-SHA256(token, nonce ‖ s)[:16]`, where `s = sprintf("%.8f,%.8f", lat, lng)` in ASCII |
| DROP | `...567b` | write | `HMAC-SHA256(token, nonce ‖ s ‖ "DROP")[:20]` with `s` exactly as above. **The drone rebuilds `s` with `%.8f` from the coordinates it parsed**, so always format with 8 decimals. |
| RESULT | `...567d` | read | ASCII outcome code (table below). **Not authenticated.** |
| **RESULT_SIGNED** | `...567e` | read | **NEW:** `code ‖ 0x00 ‖ HMAC-SHA256(token, nonce ‖ "RESULT" ‖ code)[:16]` |

- `token` = `bytes.fromhex(order.deliveryToken)` from the Firestore order
  (16 bytes), not the hex string's ASCII.
- `‖` is plain byte concatenation; `"ACK"`, `"DROP"` and `"RESULT"` are ASCII.

**Session rules (the drone enforces them):**

- One connection = one session. The session belongs to the BLE central that
  read NONCE; writes from any other connection are silently ignored.
- Write AUTH within **10 s** of reading NONCE, or the nonce expires.
- An authenticated session that is **silent for 30 s** is destroyed. Any
  read or write counts as activity, so keep polling RESULT_SIGNED at most
  every 2 s while waiting.
- **Never re-read NONCE during a delivery.** A re-read starts a new session,
  and every earlier MAC becomes invalid. After a *reconnect*, do read NONCE
  again and redo AUTH, GPS and DROP: the drone reports DESCENDING or RELEASED
  correctly on the new session.
- Send GPS about once a second. The drone needs **5 consistent fixes within
  12 s**, and a jump of more than 10 m restarts the count.

**Outcome codes** (same code in RESULT and RESULT_SIGNED):

| Code | Meaning | App behaviour |
|---|---|---|
| `PENDING` | nothing decided yet | keep going |
| `VERIFYING:n/m` | GPS fix n of m collected | show "Hold still (n/m)". Match by prefix. |
| `GEOFENCE_OK` | close enough; DROP is allowed | enable the release button |
| `GEOFENCE_FAIL` | phone is too far from the drone (12 m) | "Walk closer to the drone and hold still". The drone will not come to you. |
| `NO_DRONE_FIX` | the drone does not know its own position | "Drone is locating itself" and keep waiting; **not** the user's fault |
| `NOT_READY` | no person locked yet, or a drop request is in flight; **token not spent** | retry the same DROP every ~2 s, keep polling |
| `DESCENDING` | drop accepted; drone is moving aside and descending | "Releasing..." and keep polling |
| `RELEASED` | parcel released; token spent | success, **only if signed** (item 1) |
| `PILOT_OVERRIDE` | the pilot took control; delivery halted | stop retrying; "Delivery paused by the operator" |
| `AUTH_FAIL` | not authenticated, or the session is gone | re-handshake once (NONCE → AUTH); if it fails again, show an error |
| `SIGNATURE_FAIL` | a MAC did not verify | app bug: log it, re-handshake once, then error |
| `OK` | legacy, older drones only | treat as `GEOFENCE_OK` while waiting, never as released |

### Test vectors (must pass byte-for-byte)

```
token   000102030405060708090a0b0c0d0e0f
nonce   101112131415161718191a1b1c1d1e1f
lat,lng 12.9716, 77.5946   ->  s = "12.97160000,77.59460000"

AUTH        e76b9e0fe4021d62ea97745ef43c654dc14698aa
ACK         bc069744ca7d1c384185df67f837eed794830553
GPS write   31322e39373136303030302c37372e3539343630303030000309e9ed9ca48b73d1aa26b660663cdf
DROP        b43a55e61a3be4af0e9eca5836e2dd39b690b6e5
RESULT_SIGNED (RELEASED)   52454c45415345440022b088e68e6280748a83d2d61e49195e
RESULT_SIGNED (NOT_READY)  4e4f545f5245414459004aef21b857297aa151a662fd1455915a
```

### Changes

0. **CRITICAL, ship first: refuse any nonce that is not exactly 16 bytes.**
   - Today `drone_handshake_page.dart` signs whatever NONCE returns, after
     checking only `isNotEmpty`. A fake drone can then use the phone as a
     signing oracle:
     - nonce = `realNonce ‖ s` → the phone's AUTH *is* the GPS signature for
       any position `s` the attacker picks;
     - nonce = `realNonce ‖ s ‖ "DROP"` → the phone's AUTH *is* the DROP MAC.
   - With one device by the phone and one by the drone, a parcel can then be
     released with the recipient far away. The formal model found this
     (Verifpal, `docs/security/aerix_handshake.vp`, query `gps_sig`).
   - The drone always sends exactly 16 bytes. With the check, every MAC input
     has a different length (AUTH 16, ACK 19, GPS ≥ 37, DROP ≥ 41), so AUTH
     can never stand in for another message.
   - In `authResponse` *and* at the read site: if `nonce.length != 16`,
     disconnect, sign nothing, and log "invalid challenge length".
   - Add a test: a 37-byte nonce must throw before any HMAC is computed.

1. **Verify RESULT_SIGNED. Only a verified RELEASED is a delivery.**
   - Poll `...567e` instead of `...567d`. Split at the first `0x00`, recompute
     the tag, and compare in constant time.
   - Mark the order delivered, write the phone's receipt and show success
     **only on a verified `RELEASED`**.
   - A tag mismatch means a fake drone or a relay. Show "Could not verify the
     drone", do not mark delivered, keep the token, and log the event.
   - If `...567e` is missing (a drone not yet updated), fall back to plain
     RESULT only behind a debug flag that defaults to **off**.

2. **Verify the ACK before sending anything else.** After AUTH, read
   `...567a` and compare it to the expected ACK in constant time. On a
   mismatch, disconnect and show "This is not your delivery drone". Send no
   GPS and no DROP to an unverified drone.

3. **Follow the session rules exactly** (section above): one NONCE read per
   connection, AUTH within 10 s, polling every ≤ 2 s, never re-reading NONCE
   mid-delivery, and the full handshake again after a reconnect. Retry DROP
   on `NOT_READY` and stop on `PILOT_OVERRIDE`.

4. **Require the phone's owner before DROP.** Use `local_auth`
   (biometric or device PIN) right before the DROP write. A stolen unlocked
   phone should not release a parcel on its own (threat model T25).

5. **Token hygiene.**
   - Keep `deliveryToken` in `flutter_secure_storage`, never in plain prefs
     or logs, and delete it after a verified RELEASED or a cancellation.
   - Never log the token, the nonce, or any computed MAC (log a short SHA-256
     fingerprint if you need to correlate).

6. **Tests.**
   - Unit tests for every MAC builder against the vectors above.
   - A RESULT_SIGNED parser test: good tag, bad tag, no `0x00`, and a
     `VERIFYING:3/5` code.
   - A state-machine test: NOT_READY → retry, PILOT_OVERRIDE → stop, a
     forged RELEASED → not delivered.

7. **Evidence log.** Write a per-delivery session log on the phone and let it
   be exported:
   - timestamped connect, NONCE, AUTH, ACK-verified, each GPS write and its
     result, each DROP attempt, each RESULT_SIGNED read with its verified
     flag, and disconnects;
   - no secrets in it.

   It gets compared with the drone's `logs/ble` for the end-to-end evidence
   run.

8. **Version control.** Make sure the app is in git with a README, the
   exact Flutter version, and these protocol notes copied in as
   `docs/BLE_PROTOCOL.md`.

### Not now (needs a coordinated drone release, do not start)

- Protocol v2: explicit labels in every MAC, and a phone nonce in AUTH so the
  ACK is fresh for the phone too.
- The torch-blink code: needs a detector on the drone side first.

### Done means

All tests pass, the vectors match byte for byte, and one real delivery
against an updated drone completes with a verified RELEASED. A deliberately
corrupted RESULT_SIGNED tag must show "Could not verify the drone" and must
not mark the order delivered.
