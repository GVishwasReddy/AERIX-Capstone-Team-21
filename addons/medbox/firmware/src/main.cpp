// Medical box integrity monitor (ESP32)
// Watches temperature/humidity (DHT11), impacts and shaking (MPU6050) and the
// lid switch. Once ARMED, any violation latches the box as COMPROMISED. The
// state is stored in flash so a power cycle cannot clear it.
//
// v2 (2026-10-10): BLE peripheral for the AERIX drone. The drone is the only
// central; it reads STATUS (notify, 1 Hz + on change) and sends HMAC-signed
// commands. The drop window (ARM_DROP -> CONFIRM_DROP) suppresses impact and
// shake detection while the drone lowers and releases the box from 1 m AGL,
// then the box becomes DELIVERED, so the recipient opening the lid is
// recorded as an opening at the destination, not as tampering.
#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <mbedtls/md.h>
#include <math.h>
#include "medbox_secret.h"

#define FW_VERSION "2.0-ble"

// ---------- Pins ----------
#define PIN_SDA 21
#define PIN_SCL 22
#define PIN_DHT 4
#define PIN_LID 27  // limit switch OUT (module VCC unconnected), pull-up: LOW = lid closed
#define PIN_LED 2   // on-board LED
#define PIN_BATT 34 // battery + via 100k/100k divider (ADC1, input-only pin)
#define MPU_ADDR 0x68

// ---------- BLE (must match drone_stack/payload/medbox_protocol.py) ----------
#define SVC_UUID    "aad1880d-ae20-4b48-8f1b-79b6bb371248"
#define STATUS_UUID "d3bb4bf2-2c47-4630-82fb-ffe15dd26718"  // read + notify, compact JSON
#define NONCE_UUID  "67fc87e8-19b3-47fb-90d7-207717604167"  // read, 16 hex chars, single use
#define CMD_UUID    "a47403e2-3eef-4efa-92fc-d8685a4dd86b"  // write "CMD|nonce|hmac32hex"

// ---------- Tunables ----------
const uint32_t ARM_HOLD_MS      = 3000;  // lid must stay closed this long before arming
const uint32_t LID_DEBOUNCE_MS  = 50;
const uint32_t IMU_PERIOD_MS    = 10;    // 100 Hz
// Motion limits for a drone payload. Tune from a real flight: fly with the box ARMED,
// read max_shake / max_shock from the report, then set each limit to ~2x those values.
const float    SHOCK_G          = 3.0;   // instantaneous deviation from gravity = impact/drop
const float    SHAKE_RMS_G      = 0.6;   // RMS deviation over a window = vigorous shaking
const uint32_t SHAKE_WINDOW_MS  = 2000;
const uint8_t  SHAKE_WINDOWS_TO_TRIP = 2;  // consecutive bad windows (4 s) so gusts/landings don't trip
const uint32_t DHT_PERIOD_MS    = 2000;  // DHT11 cannot be read faster than ~1 Hz
const uint8_t  SENSOR_MAX_FAILS = 5;     // consecutive failed reads = sensor fault
const uint32_t REPORT_PERIOD_MS = 5000;
const uint32_t BLE_STATUS_MS    = 1000;  // STATUS notify period while the drone is connected

// Drop window. ARM_DROP arrives while the drone approaches/descends (seconds
// before the servo fires: the box falls 1 m in ~0.45 s, faster than the Pi
// can see drop_stage=release and say so). CONFIRM_DROP arrives once the
// navigator reports the release. CANCEL_DROP when the drop aborts.
const uint32_t DROP_EXPECT_MAX_MS = 90000;  // > navigator drop_timeout_s (45 s) + approach
const uint32_t DROP_LANDING_MS    = 5000;   // after CONFIRM: fall + bounce + settle, then DELIVERED

// Storage conditions: set these for the medicine being carried
// (2-8 C cold chain such as insulin/vaccines, 15-25 C room temperature, 30 C = USP excursion limit)
const float    TEMP_MIN_C   = 2.0;
const float    TEMP_MAX_C   = 30.0;
const float    HUMIDITY_MAX = 70.0;
const uint32_t EXCURSION_MS = 60000;  // out of range this long = unsafe (filters DHT11 glitches)

// 1S LiPo (3.7 V 1000 mAh) through TP4056
const float    BATT_DIVIDER    = 2.0;   // (R1 + R2) / R2
const float    BATT_LOW_V      = 3.50;  // ~10% left: warn
const float    BATT_CRITICAL_V = 3.30;  // monitoring about to stop
const float    BATT_RECOVER_V  = 3.65;  // hysteresis: low warning clears once charged past this
const float    BATT_MISSING_V  = 2.50;  // below this the divider is not connected
const uint32_t BATT_PERIOD_MS  = 5000;

// Numeric codes are what goes over BLE (a notification is capped at MTU-3
// bytes); the Pi maps them back with the same tables. APPEND ONLY.
enum State : uint8_t { UNARMED, ARMED, COMPROMISED, DELIVERED };
// R_SENSOR_FAULT value: 1 = MPU6050 not answering, 2 = DHT11 not answering, 3 = MPU6050 reset
enum Reason : uint8_t { R_NONE, R_LID_OPENED, R_TEMP, R_HUMIDITY, R_SHOCK, R_SHAKE, R_SENSOR_FAULT,
                        R_DROP_UNCONFIRMED };
const char* STATE_NAMES[]  = {"UNARMED", "ARMED", "COMPROMISED", "DELIVERED"};
const char* REASON_NAMES[] = {"none", "lid opened", "temperature out of range",
                              "humidity out of range", "impact/drop", "vigorous shaking",
                              "sensor fault", "impact in an unconfirmed drop"};

enum BattLevel : uint8_t { BATT_UNKNOWN, BATT_OK, BATT_LOW, BATT_CRITICAL };
const char* BATT_NAMES[] = {"n/a", "ok", "LOW", "CRITICAL"};

enum DropPhase : uint8_t { DROP_NONE, DROP_EXPECTING, DROP_LANDING };
const char* DROP_NAMES[] = {"none", "expecting", "landing"};

// What an expired, never-confirmed drop window turns into (resolveExpiredDropWindow).
enum DropVerdict : uint8_t { VERDICT_REARM, VERDICT_DELIVERED, VERDICT_COMPROMISED };

DHT dht(PIN_DHT, DHT11);
Preferences prefs;

// Persisted state
State state = UNARMED;
Reason reason = R_NONE;
float reasonValue = 0;
uint32_t compromisedAfterS = 0;  // seconds armed (since last boot) when the violation happened
uint32_t bootsWhileArmed = 0;    // power interruptions while sealed
bool dropConfirmed = false;      // CONFIRM_DROP received; survives a landing brownout
bool openedAtDest = false;       // lid opened after DELIVERED (expected: the recipient)
float landingPeakG = 0;          // worst deviation inside the drop window

// Live readings
float tempC = NAN, humidity = NAN, shakeRms = 0, peakShock = 0;
float maxShake = 0, maxShock = 0;  // worst seen since arming (for threshold tuning)
uint8_t badShakeWindows = 0;
uint32_t armedAtMs = 0;
uint8_t imuFails = 0, dhtFails = 0;
uint32_t tempBadSinceMs = 0, humBadSinceMs = 0;  // start of current excursion, 0 = in range
float battV = NAN;
BattLevel battLevel = BATT_UNKNOWN;

// Drop window
DropPhase dropPhase = DROP_NONE;
uint32_t dropDeadlineMs = 0;       // EXPECTING: give up waiting for CONFIRM at this time
uint32_t landingUntilMs = 0;       // LANDING: become DELIVERED at this time

// Lid debounce
bool lidClosed = false, lidRaw = false;
uint32_t lidRawChangedMs = 0, lidClosedSinceMs = 0;

// Shake window
float gravityBaseline = NAN;
double sumSq = 0;
uint32_t nSamples = 0, windowStartMs = 0;

// BLE
NimBLECharacteristic *statusChr = nullptr, *nonceChr = nullptr;
volatile bool bleConnected = false;
bool statusDirty = true;
uint32_t statusSeq = 0;
char nonceHex[17] = {0};
uint8_t hmacKey[32];
size_t hmacKeyLen = 0;
char lastAck[40] = "";
uint16_t ackCount = 0;
// The CMD write callback runs on the NimBLE host task; it only parks the text
// here and loop() executes it, so no state is ever touched from two tasks.
portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;
char pendingCmd[128];
volatile bool cmdPending = false;

// ---------- Persistence ----------
void saveState() {
  prefs.putUChar("state", state);
  prefs.putUChar("reason", reason);
  prefs.putFloat("value", reasonValue);
  prefs.putUInt("afterS", compromisedAfterS);
  prefs.putUInt("boots", bootsWhileArmed);
  prefs.putBool("dropConf", dropConfirmed);
  prefs.putBool("opened", openedAtDest);
  prefs.putFloat("landG", landingPeakG);
}

void loadState() {
  state = (State)prefs.getUChar("state", UNARMED);
  reason = (Reason)prefs.getUChar("reason", R_NONE);
  reasonValue = prefs.getFloat("value", 0);
  compromisedAfterS = prefs.getUInt("afterS", 0);
  bootsWhileArmed = prefs.getUInt("boots", 0);
  dropConfirmed = prefs.getBool("dropConf", false);
  openedAtDest = prefs.getBool("opened", false);
  landingPeakG = prefs.getFloat("landG", 0);
}

// ---------- Alerts ----------
// Single exit point for every alert. Serial + an immediate BLE STATUS push
// (the drone's bridge turns state changes into GCS console lines and toasts).
void notify(const char* event, const char* detail) {
  Serial.printf("[ALERT] %s: %s\n", event, detail);
  statusDirty = true;
}

int battPercent(float v) {  // rough 1S LiPo discharge curve under light load
  static const float V[] = {3.30, 3.50, 3.60, 3.70, 3.80, 3.90, 4.00, 4.10, 4.20};
  static const int   P[] = {0, 10, 20, 40, 55, 70, 80, 90, 100};
  if (v <= V[0]) return 0;
  for (int i = 1; i < 9; i++)
    if (v < V[i]) return P[i - 1] + (P[i] - P[i - 1]) * (v - V[i - 1]) / (V[i] - V[i - 1]);
  return 100;
}

// ---------- State machine ----------
void report() {
  Serial.printf("[%s] T=%.1fC H=%.0f%% g=%.2f shake=%.3fg peak=%.3fg lid=%s",
                STATE_NAMES[state], tempC, humidity, gravityBaseline, shakeRms, peakShock,
                lidClosed ? "closed" : "OPEN");
  if (imuFails) Serial.printf(" IMU_FAIL=%u", imuFails);
  if (dhtFails) Serial.printf(" DHT_FAIL=%u", dhtFails);
  if (state == ARMED) Serial.printf(" armed=%lus max_shake=%.2fg max_shock=%.2fg",
                                    (millis() - armedAtMs) / 1000, maxShake, maxShock);
  if (dropPhase != DROP_NONE) Serial.printf(" drop=%s landing_peak=%.2fg", DROP_NAMES[dropPhase], landingPeakG);
  if (state == COMPROMISED)
    Serial.printf(" reason=\"%s\" value=%.2f at=%lus", REASON_NAMES[reason], reasonValue, compromisedAfterS);
  if (state == DELIVERED)
    Serial.printf(" landing_peak=%.2fg opened=%s", landingPeakG, openedAtDest ? "yes" : "no");
  if (battLevel == BATT_UNKNOWN) Serial.print(" bat=n/a");
  else Serial.printf(" bat=%.2fV(%d%%,%s)", battV, battPercent(battV), BATT_NAMES[battLevel]);
  if (bootsWhileArmed) Serial.printf(" power_interruptions=%lu", bootsWhileArmed);
  Serial.printf(" ble=%s", bleConnected ? "drone" : "adv");
  Serial.println();
}

void compromise(Reason r, float value) {
  if (state != ARMED) return;  // only a sealed box can be compromised; first reason wins
  state = COMPROMISED;
  reason = r;
  reasonValue = value;
  compromisedAfterS = (millis() - armedAtMs) / 1000;
  dropPhase = DROP_NONE;
  dropConfirmed = false;
  saveState();
  char detail[96];
  snprintf(detail, sizeof detail, "%s (value %.2f) after %lus armed - medicine UNSAFE",
           REASON_NAMES[r], value, compromisedAfterS);
  notify("BOX_COMPROMISED", detail);
}

bool sensorsHealthy() {
  return !isnan(tempC) && imuFails == 0 && dhtFails < SENSOR_MAX_FAILS;
}

void clearDrop() {
  dropPhase = DROP_NONE;
  dropConfirmed = false;
  openedAtDest = false;
  landingPeakG = 0;
}

void arm() {
  state = ARMED;
  reason = R_NONE;
  reasonValue = 0;
  compromisedAfterS = 0;
  bootsWhileArmed = 0;
  armedAtMs = millis();
  tempBadSinceMs = humBadSinceMs = 0;
  maxShake = maxShock = 0;
  badShakeWindows = 0;
  clearDrop();
  saveState();
  statusDirty = true;
  Serial.println("[MEDBOX] Lid sealed -> ARMED. Monitoring started.");
}

// Sealed box handed over intact: monitoring responsibility ends here.
void deliver() {
  if (state != ARMED) return;
  state = DELIVERED;
  dropPhase = DROP_NONE;
  dropConfirmed = false;
  reasonValue = landingPeakG;
  saveState();
  char detail[64];
  snprintf(detail, sizeof detail, "intact at handover, landing peak %.2f g", landingPeakG);
  notify("BOX_DELIVERED", detail);
}

void resetBox(uint32_t now, const char* who) {
  state = UNARMED;
  reason = R_NONE;
  reasonValue = 0;
  compromisedAfterS = 0;
  bootsWhileArmed = 0;
  clearDrop();
  lidClosedSinceMs = now;  // require a fresh ARM_HOLD_MS before re-arming
  saveState();
  statusDirty = true;
  Serial.printf("[MEDBOX] Reset to UNARMED (%s). Keep lid closed to re-arm.\n", who);
}

// ---------- Lid ----------
void updateLid(uint32_t now) {
  bool raw = digitalRead(PIN_LID) == LOW;
  if (raw != lidRaw) { lidRaw = raw; lidRawChangedMs = now; }
  if (lidRaw != lidClosed && now - lidRawChangedMs >= LID_DEBOUNCE_MS) {
    lidClosed = lidRaw;
    statusDirty = true;
    if (lidClosed) lidClosedSinceMs = now;
    Serial.printf("[LID] %s\n", lidClosed ? "closed" : "OPENED");
    if (!lidClosed) {
      // Released and on the ground: the next hand on the lid is the recipient's.
      if (state == ARMED && dropPhase == DROP_LANDING) deliver();
      if (state == DELIVERED) {
        if (!openedAtDest) {
          openedAtDest = true;
          saveState();
          notify("OPENED_AT_DESTINATION", "lid opened after delivery");
        }
      } else {
        compromise(R_LID_OPENED, 0);
      }
    }
  }
  if (state == UNARMED && lidClosed && now - lidClosedSinceMs >= ARM_HOLD_MS && sensorsHealthy())
    arm();
}

// ---------- MPU6050 ----------
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

uint8_t mpuRead8(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
}

bool mpuInit() {
  return mpuWrite(0x6B, 0x00)    // wake from sleep
      && mpuWrite(0x1A, 0x04)    // digital low-pass ~21 Hz: passes hand shaking, rejects prop vibration
      && mpuWrite(0x1C, 0x10);   // accel range +-8 g (default +-2 g would clip impacts)
}

bool mpuReadAccel(float a[3]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)6) != 6) return false;
  int16_t raw[3];
  for (int i = 0; i < 3; i++) raw[i] = (Wire.read() << 8) | Wire.read();
  if (raw[0] == 0 && raw[1] == 0 && raw[2] == 0) return false;  // sleeping MPU, not real data
  for (int i = 0; i < 3; i++) a[i] = raw[i] / 4096.0f;
  return true;
}

// A brief power glitch (e.g. a jumper jolted loose) resets the MPU6050 into sleep
// mode with default config: it still ACKs on I2C but reads all zeros. Detect the
// reset, wake it again, and condemn the box if it happened while sealed.
void checkMpuConfig() {
  uint8_t pwr = mpuRead8(0x6B), acc = mpuRead8(0x1C);
  if (pwr == 0xFF || acc == 0xFF) return;  // bus error: counted by the read-failure path
  if (pwr == 0x00 && acc == 0x10) return;
  Serial.printf("[MPU6050] reset detected (PWR=0x%02X ACC=0x%02X), re-initialising\n", pwr, acc);
  mpuInit();
  gravityBaseline = NAN;
  compromise(R_SENSOR_FAULT, 3);
}

void sampleImu(uint32_t now) {
  float a[3];
  if (!mpuReadAccel(a)) {
    if (++imuFails >= SENSOR_MAX_FAILS) compromise(R_SENSOR_FAULT, 1);
    return;
  }
  imuFails = 0;
  float mag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  if (isnan(gravityBaseline)) gravityBaseline = mag;
  gravityBaseline += 0.01f * (mag - gravityBaseline);  // slow tracker absorbs gravity + sensor offset
  float dev = fabsf(mag - gravityBaseline);
  if (dev > peakShock) peakShock = dev;

  // Inside the drop window the jolts are the delivery itself: record them as the
  // landing, keep them out of the flight-tuning maxima, and do not trip on them.
  bool dropping = state == ARMED && dropPhase != DROP_NONE;
  if (dropping) {
    if (dev > landingPeakG) landingPeakG = dev;
  } else {
    if (state == ARMED && dev > maxShock) maxShock = dev;
    if (dev > SHOCK_G) compromise(R_SHOCK, dev);
  }

  sumSq += dev * dev;
  nSamples++;
  if (now - windowStartMs >= SHAKE_WINDOW_MS) {
    shakeRms = sqrt(sumSq / nSamples);
    if (dropping) {
      badShakeWindows = 0;
    } else {
      if (state == ARMED && shakeRms > maxShake) maxShake = shakeRms;
      badShakeWindows = shakeRms > SHAKE_RMS_G ? badShakeWindows + 1 : 0;
      if (badShakeWindows >= SHAKE_WINDOWS_TO_TRIP) compromise(R_SHAKE, shakeRms);
    }
    sumSq = 0;
    nSamples = 0;
    windowStartMs = now;
  }
}

// ---------- DHT11 ----------
// Called on every successful DHT11 read while the box is ARMED.
// Return R_NONE while conditions are acceptable, or R_TEMP / R_HUMIDITY once
// the medicine should be considered unsafe.
// An excursion must persist for EXCURSION_MS so a single bad DHT11 reading
// cannot condemn the medicine.
Reason evaluateEnvironment(float t, float h, uint32_t now) {
  bool tempBad = t < TEMP_MIN_C || t > TEMP_MAX_C;
  bool humBad = h > HUMIDITY_MAX;
  if (tempBad && !tempBadSinceMs) { tempBadSinceMs = now; notify("TEMP_EXCURSION", "temperature out of range"); }
  if (humBad && !humBadSinceMs) { humBadSinceMs = now; notify("HUMIDITY_EXCURSION", "humidity out of range"); }
  if (!tempBad) tempBadSinceMs = 0;
  if (!humBad) humBadSinceMs = 0;
  if (tempBadSinceMs && now - tempBadSinceMs >= EXCURSION_MS) return R_TEMP;
  if (humBadSinceMs && now - humBadSinceMs >= EXCURSION_MS) return R_HUMIDITY;
  return R_NONE;
}

void sampleDht(uint32_t now) {
  float t = dht.readTemperature(), h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {
    if (++dhtFails >= SENSOR_MAX_FAILS) compromise(R_SENSOR_FAULT, 2);
    return;
  }
  dhtFails = 0;
  tempC = t;
  humidity = h;
  if (state == ARMED) {
    Reason r = evaluateEnvironment(t, h, now);
    if (r != R_NONE) compromise(r, r == R_TEMP ? t : h);
  }
}

// ---------- Battery ----------
void sampleBattery() {
  uint32_t mv = 0;
  for (int i = 0; i < 16; i++) mv += analogReadMilliVolts(PIN_BATT);
  battV = mv / 16 / 1000.0f * BATT_DIVIDER;

  BattLevel lvl;
  if (battV < BATT_MISSING_V) lvl = BATT_UNKNOWN;
  else if (battV < BATT_CRITICAL_V) lvl = BATT_CRITICAL;
  else if (battV < BATT_LOW_V) lvl = BATT_LOW;
  else if (battV < BATT_RECOVER_V && battLevel >= BATT_LOW) lvl = BATT_LOW;
  else lvl = BATT_OK;
  if (lvl == battLevel) return;

  char detail[48];
  snprintf(detail, sizeof detail, "%.2fV (%d%%)", battV, battPercent(battV));
  if (lvl == BATT_CRITICAL) notify("BATTERY_CRITICAL", detail);
  else if (lvl == BATT_LOW) notify("BATTERY_LOW", detail);
  else if (lvl == BATT_OK && battLevel >= BATT_LOW) notify("BATTERY_OK", detail);
  battLevel = lvl;
}

// ---------- Drop window ----------
// The window opened (ARM_DROP) but neither CONFIRM_DROP nor CANCEL_DROP ever
// arrived before DROP_EXPECT_MAX_MS ran out - typically the BLE link to the
// drone dropped mid-delivery. Impact and shake detection were suppressed the
// whole time, so this is the one moment the box must decide on its own what
// those suppressed seconds meant.
//   landingPeakG - worst |a| deviation seen inside the window (SHOCK_G = 3.0 is
//                  the normal impact limit; a 1 m release lands far above it)
//   lidClosed    - lid state right now
//   tempOk       - no temperature/humidity excursion is in progress
// Return VERDICT_REARM (resume normal monitoring), VERDICT_DELIVERED (treat it
// as a completed drop) or VERDICT_COMPROMISED (latch R_DROP_UNCONFIRMED).
DropVerdict resolveExpiredDropWindow(float landingPeakG, bool lidClosed, bool tempOk) {
  // TODO(human): decide what an unconfirmed drop window means.
  return VERDICT_REARM;
}

void updateDrop(uint32_t now) {
  if (state != ARMED) return;
  if (dropPhase == DROP_LANDING && (int32_t)(now - landingUntilMs) >= 0) {
    deliver();
  } else if (dropPhase == DROP_EXPECTING && (int32_t)(now - dropDeadlineMs) >= 0) {
    bool tempOk = !tempBadSinceMs && !humBadSinceMs;
    DropVerdict v = resolveExpiredDropWindow(landingPeakG, lidClosed, tempOk);
    char detail[80];
    snprintf(detail, sizeof detail, "no confirm in %lus, peak %.2f g -> %s", DROP_EXPECT_MAX_MS / 1000,
             landingPeakG, v == VERDICT_REARM ? "re-armed" : v == VERDICT_DELIVERED ? "delivered" : "compromised");
    notify("DROP_WINDOW_EXPIRED", detail);
    dropPhase = DROP_NONE;
    if (v == VERDICT_DELIVERED) {
      deliver();
    } else if (v == VERDICT_COMPROMISED) {
      compromise(R_DROP_UNCONFIRMED, landingPeakG);
    } else {
      gravityBaseline = NAN;  // re-learn gravity: the window may have left it skewed
      landingPeakG = 0;
    }
  }
}

// ---------- BLE commands ----------
void newNonce() {
  uint32_t a = esp_random(), b = esp_random();
  snprintf(nonceHex, sizeof nonceHex, "%08lx%08lx", (unsigned long)a, (unsigned long)b);
  // Explicit length: the templated setValue(char[17]) would publish sizeof() =
  // 17 bytes including the NUL, the Pi echoes it back inside the command, and
  // strlen() then cuts the command off before its signature.
  if (nonceChr) nonceChr->setValue((const uint8_t*)nonceHex, 16);
}

int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

void loadKey() {
  const char* k = MEDBOX_KEY_HEX;
  size_t n = strlen(k) / 2;
  if (n > sizeof hmacKey) n = sizeof hmacKey;
  for (size_t i = 0; i < n; i++) hmacKey[i] = (hexNibble(k[2 * i]) << 4) | hexNibble(k[2 * i + 1]);
  hmacKeyLen = n;
}

// HMAC-SHA256(key, nonce "|" cmd), first 16 bytes as 32 lowercase hex chars.
void signFor(const char* cmd, char out[33]) {
  char msg[96];
  snprintf(msg, sizeof msg, "%s|%s", nonceHex, cmd);
  uint8_t mac[32];
  mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), hmacKey, hmacKeyLen,
                  (const uint8_t*)msg, strlen(msg), mac);
  for (int i = 0; i < 16; i++) snprintf(out + 2 * i, 3, "%02x", mac[i]);
}

bool sigEqual(const char* a, const char* b) {  // constant time over 32 chars
  uint8_t d = 0;
  for (int i = 0; i < 32; i++) d |= (uint8_t)(a[i] ^ b[i]);
  return d == 0 && strlen(a) == 32 && strlen(b) == 32;
}

void setAck(const char* cmd, const char* result) {
  snprintf(lastAck, sizeof lastAck, "%s:%s", cmd, result);
  ackCount++;
  statusDirty = true;
  Serial.printf("[BLE] %s\n", lastAck);
}

const char* runCommand(const char* cmd, uint32_t now) {
  if (!strcmp(cmd, "ARM_DROP")) {
    if (state != ARMED) return "BAD_STATE";
    if (dropPhase == DROP_LANDING) return "ALREADY_RELEASED";
    if (dropPhase == DROP_NONE) landingPeakG = 0;
    dropPhase = DROP_EXPECTING;
    dropDeadlineMs = now + DROP_EXPECT_MAX_MS;  // a repeated ARM_DROP extends the window
    return "OK";
  }
  if (!strcmp(cmd, "CONFIRM_DROP")) {
    if (state == DELIVERED) return "OK";        // idempotent: a retry after the ack was lost
    if (state != ARMED) return "BAD_STATE";     // e.g. already COMPROMISED by an unannounced impact
    if (dropPhase != DROP_LANDING) {
      dropPhase = DROP_LANDING;
      landingUntilMs = now + DROP_LANDING_MS;
      dropConfirmed = true;
      saveState();                              // a landing brownout must not lose this
    }
    return "OK";
  }
  if (!strcmp(cmd, "CANCEL_DROP")) {
    if (dropPhase == DROP_LANDING) return "ALREADY_RELEASED";
    if (dropPhase == DROP_EXPECTING) {
      dropPhase = DROP_NONE;
      landingPeakG = 0;
      gravityBaseline = NAN;
    }
    return "OK";
  }
  if (!strcmp(cmd, "RESET")) {
    resetBox(now, "BLE");
    return "OK";
  }
  return "UNKNOWN";
}

void handleBleCommand(uint32_t now) {
  if (!cmdPending) return;
  char buf[128];
  portENTER_CRITICAL(&cmdMux);
  strncpy(buf, pendingCmd, sizeof buf);
  buf[sizeof buf - 1] = 0;
  cmdPending = false;
  portEXIT_CRITICAL(&cmdMux);

  // "CMD|nonce|sig"
  char* cmd = buf;
  char* p1 = strchr(buf, '|');
  char* p2 = p1 ? strchr(p1 + 1, '|') : nullptr;
  if (!p1 || !p2) {
    Serial.printf("[BLE] malformed command: %u bytes \"%.70s\"\n", (unsigned)strlen(buf), buf);
    setAck("?", "MALFORMED");
    newNonce();
    return;
  }
  *p1 = 0; *p2 = 0;
  const char* nonce = p1 + 1;
  const char* sig = p2 + 1;
  char want[33];
  signFor(cmd, want);
  bool ok = !strcmp(nonce, nonceHex) && sigEqual(sig, want);
  newNonce();  // single use, whether or not it verified: no replay, no brute-forcing one nonce
  if (!ok) { setAck(cmd, "BAD_SIG"); return; }
  setAck(cmd, runCommand(cmd, now));
}

class CmdCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) override {
    std::string v = c->getValue();
    portENTER_CRITICAL(&cmdMux);
    size_t n = v.size() < sizeof pendingCmd - 1 ? v.size() : sizeof pendingCmd - 1;
    memcpy(pendingCmd, v.data(), n);
    pendingCmd[n] = 0;
    cmdPending = true;
    portEXIT_CRITICAL(&cmdMux);
  }
};

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, ble_gap_conn_desc* desc) override {
    // One central only: the drone. A second (a curious phone) is dropped, so it
    // can neither crowd the drone's link nor queue commands behind it.
    if (s->getConnectedCount() > 1) { s->disconnect(desc->conn_handle); return; }
    bleConnected = true;
    statusDirty = true;
    newNonce();
  }
  void onDisconnect(NimBLEServer* s, ble_gap_conn_desc* desc) override {
    bleConnected = s->getConnectedCount() > 0;
    newNonce();  // NimBLE re-advertises on its own (advertiseOnDisconnect)
  }
};

// Compact JSON with 1-2 letter keys: a notification is capped at MTU-3 bytes.
// Keys are decoded in drone_stack/payload/medbox_protocol.py - keep in step.
void pushStatus(uint32_t now) {
  if (!statusChr) return;
  char js[240];
  int n = snprintf(js, sizeof js,
      "{\"v\":2,\"q\":%lu,\"s\":%u,\"r\":%u,\"rv\":%.2f,\"t\":%.1f,\"h\":%.0f,\"sh\":%.3f,"
      "\"pk\":%.2f,\"ms\":%.2f,\"mk\":%.2f,\"l\":%u,\"up\":%lu,\"bv\":%.2f,\"bl\":%u,\"pi\":%lu,"
      "\"d\":%u,\"lg\":%.2f,\"o\":%u,\"f\":%u,\"a\":\"%s\",\"an\":%u}",
      (unsigned long)++statusSeq, state, reason, reasonValue,
      isnan(tempC) ? -99.0f : tempC, isnan(humidity) ? -1.0f : humidity, shakeRms,
      peakShock, maxShake, maxShock, lidClosed ? 1 : 0,
      state == ARMED ? (unsigned long)((now - armedAtMs) / 1000) : 0UL,
      battLevel == BATT_UNKNOWN ? 0.0f : battV, battLevel, (unsigned long)bootsWhileArmed,
      dropPhase, landingPeakG, openedAtDest ? 1 : 0,
      (imuFails ? 1 : 0) | (dhtFails ? 2 : 0), lastAck, ackCount);
  if (n <= 0 || n >= (int)sizeof js) return;
  statusChr->setValue((uint8_t*)js, n);
  if (bleConnected) statusChr->notify();
  peakShock = 0;  // "pk" is the worst since the previous status
  statusDirty = false;
}

void setupBle() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  char name[16];
  snprintf(name, sizeof name, "MEDBOX-%02X%02X", mac[4], mac[5]);
  NimBLEDevice::init(name);
  NimBLEDevice::setMTU(247);
  NimBLEServer* srv = NimBLEDevice::createServer();
  srv->setCallbacks(new ServerCallbacks());
  NimBLEService* svc = srv->createService(SVC_UUID);
  statusChr = svc->createCharacteristic(STATUS_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  nonceChr = svc->createCharacteristic(NONCE_UUID, NIMBLE_PROPERTY::READ);
  NimBLECharacteristic* cmdChr = svc->createCharacteristic(CMD_UUID, NIMBLE_PROPERTY::WRITE);
  cmdChr->setCallbacks(new CmdCallbacks());
  svc->start();
  newNonce();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);  // the name rides in the scan response
  adv->setMinInterval(320);    // 200-300 ms (0.625 ms units): battery over connect latency
  adv->setMaxInterval(480);
  adv->start();
  Serial.printf("[BLE] advertising as %s\n", name);
}

// ---------- Serial commands ----------
void handleSerial(uint32_t now) {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();
  if (cmd == "RESET") {
    resetBox(now, "serial");  // physical access = authorised
  } else if (cmd == "STATUS") {
    report();
  } else if (cmd.length()) {
    Serial.println("Commands: STATUS, RESET");
  }
}

// ---------- LED ----------
// solid = armed, slow blink = unarmed, fast blink = compromised, double blink = delivered
void updateLed(uint32_t now) {
  bool on;
  uint32_t ph = now % 1000;
  switch (state) {
    case ARMED:       on = true; break;
    case UNARMED:     on = (now / 500) % 2; break;
    case COMPROMISED: on = (now / 100) % 2; break;
    default:          on = ph < 100 || (ph >= 200 && ph < 300); break;
  }
  digitalWrite(PIN_LED, on);
}

void setup() {
  setCpuFrequencyMhz(80);  // ~half the current draw of 240 MHz; the radio's minimum
  Serial.begin(115200);
  Serial.setTimeout(50);
  pinMode(PIN_LID, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  Wire.begin(PIN_SDA, PIN_SCL);
  dht.begin();
  prefs.begin("medbox", false);
  loadState();
  loadKey();
  delay(1000);

  Serial.println("\n===== MEDBOX MONITOR " FW_VERSION " =====");
  if (!mpuInit()) Serial.println("[MPU6050] not responding");

  lidRaw = lidClosed = digitalRead(PIN_LID) == LOW;
  lidClosedSinceMs = millis();
  if (state == ARMED) {
    bootsWhileArmed++;
    armedAtMs = millis();
    saveState();
    Serial.println("[MEDBOX] Rebooted while ARMED (power interruption logged)");
    if (dropConfirmed) {
      // The release was confirmed before the reboot - the landing jolt most
      // likely browned us out. Finish the delivery rather than re-arm.
      deliver();
      if (!lidClosed) { openedAtDest = true; saveState(); }
    } else if (!lidClosed) {
      compromise(R_LID_OPENED, 0);
    }
  }
  windowStartMs = millis();
  sampleBattery();
  setupBle();
  report();
}

void loop() {
  static uint32_t lastImu = 0, lastMpuCheck = 0, lastDht = 0, lastBatt = 0, lastReport = 0, lastStatus = 0;
  uint32_t now = millis();

  updateLid(now);
  if (now - lastImu >= IMU_PERIOD_MS) { lastImu = now; sampleImu(now); }
  if (now - lastMpuCheck >= 1000) { lastMpuCheck = now; checkMpuConfig(); }
  if (now - lastDht >= DHT_PERIOD_MS) { lastDht = now; sampleDht(now); }
  if (now - lastBatt >= BATT_PERIOD_MS) { lastBatt = now; sampleBattery(); }
  if (now - lastReport >= REPORT_PERIOD_MS) { lastReport = now; report(); }
  updateDrop(now);
  handleBleCommand(now);
  // On change at most every 100 ms, otherwise once a second.
  if ((statusDirty && now - lastStatus >= 100) || now - lastStatus >= BLE_STATUS_MS) {
    lastStatus = now;
    pushStatus(now);
  }
  handleSerial(now);
  updateLed(now);
}
