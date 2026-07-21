//  Valve Controller — DEEP-SLEEP DEPLOYMENT BUILD
//  ESP32 DevKitC-32UE · 2× PCA9685 (0x40 / 0x41) · 21 Hiwonder HPS-2018 valves
//  (main intake = Ch20 · sample valves = Ch0–19)
//
//  NOTES
//  ────────────────────────────────────────────────────────────────────────
//
//  1. PASSIVE HOLD:
//     Between moves the code cuts PWM (full-OFF) and lets each valve hold its
//     position by gear friction / self-locking — exactly as the original did
//     during its 8 h waits.  The whole low-power story depends on the HPS-2018
//     gear train NOT back-driving under a pinched tube at ~31 bar, in BOTH the
//     main-open and sample-closed positions.  >>> CONFIRM ON THE BENCH UNDER
//     LOAD before trusting it for 18 h.  If a valve creeps, you lose sample
//     integrity silently.  If it back-drives, that valve must stay energized —
//     which the 21-day power budget cannot support, so it becomes a mechanical
//     fix, not a firmware one.
//
//  2. FLUIDIC SEQUENCE: During each long wait the MAIN valve is OPEN (manifold
//     flushing); to sample, we close main, open sample N for SERVO_OPEN_TIME_MS,
//     close sample N, reopen main.
//
//  3. INTER-SAMPLE INTERVAL default = 18 h (per the mission spec).
//
//  4. BATTERY CUTOFF = 5.60 V (rested).  Below this at wake we PARK and do NOT
//     actuate, to avoid a servo inrush browning out the rail mid-register-write
//     (the failure you already hit once).  This trades late samples for pack
//     protection.  Tune VBATT_CUTOFF_V, or flip the policy, with Howard.
//
//  5. RESET BEHAVIOR while armed:
//      • Deep-sleep timer wake  → normal mission cadence.
//      • BROWNOUT at depth      → resume mission, WiFi stays OFF, re-establish
//                               valve state, sleep one fresh interval before
//                               the pending sample (avoids double-sampling).
//      • Any other fresh reset (power-on / EN button) → enter ARM MODE (WiFi
//        up) so an operator on the bench can reconfigure.  It does NOT silently
//        resume the mission.
//
//  6. SERVO-RAIL GATE (Q2):  Optional.  If you have a GPIO wired to your Q2
//  gate-drive circuit, define SERVO_RAIL_EN_PIN to also cut the servo rail
//  during sleep (extra margin).  It is NOT required to hit the power budget:
//  with the radios off, the PCA9685s slept, and the servos limp (no signal),
//  the ESP32 core + peripherals already sit at their floor.
//
//  7. SLEEP-CURRENT NOTE: this firmware gets the ESP32 core and both PCA9685s down
//  to microamps.  The residual ~10 mA on a stock DevKitC is the onboard AMS1117
//  LDO + CP2102 USB chip — hardware, not firmware.  If you need lower, power the
//  module directly / use a bare WROOM-32UE, or remove those parts.
//
//  8. BENCH TESTING & CALIBRATION HANDOFF
//  -----------------------------------
//  A web "Testing Mode" (toggle in the UI) lets you exercise one servo at a
//  time as you wire each channel: open / close / 90° / sweep / release, a raw-
//  angle jog for checking horn clocking, and a hold-check that drives the valve
//  then cuts PWM so you can watch for creep.  Arming is locked out while testing
//  mode is on (and vice-versa) so you can't deploy by accident on the bench.
//
//  Open/close angles are still set by calibration sketch — this
//  firmware only STORES the result in CH_OPEN_DEG[] + CLOSE_OFFSET_DEG.  Those
//  numbers only transfer correctly if that sketch uses the SAME angle→pulse
//  mapping this file does: PCA9685 @ 50 Hz, counts 102–512 over 0–180°
//  (≈500–2500 µs), Adafruit lib oscillator default 25 MHz.  If your sketch uses
//  the Servo library, writeMicroseconds(), a different µs range, or a different
//  frequency, a "95°" there will NOT point the horn where "95°" does here.


#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <driver/gpio.h>

// ── Network (ARM MODE only — never started on a mission wake) ────────────────
const char* AP_SSID     = "LanderController";
const char* AP_PASSWORD = "lander1234";

// ── Pins ─────────────────────────────────────────────────────────────────────
#define ARM_LED_PIN      27       // GPIO27 → 2.2k → LED1 (red/amber/yellow-green)
#define VBATT_SENSE_PIN  34       // GPIO34, ADC1_CH6, 220k/100k divider
// Optional high-side servo-rail enable. Uncomment + set polarity to match your
// Q2 gate-drive circuit if you want the rail cut during deep sleep.
// #define SERVO_RAIL_EN_PIN        <gpio>
// #define SERVO_RAIL_EN_ACTIVE_HIGH 1

// ── Battery sense ─────────────────────────────────────────────────────────────
const float   VBATT_DIVIDER_RATIO = 0.3125f;  // Vadc = 0.3125 * Vbatt
const float   VBATT_CUTOFF_V      = 5.60f;     // park below this (see decision #4)
const float   VBATT_PLAUSIBLE_MIN = 3.00f;     // below → assume sense fault, ignore
const uint8_t VBATT_SAMPLES       = 16;

// ── ARM confirmation LED window (seal the enclosure during this blink) ────────
const uint32_t ARM_BLINK_MS = 45000;           // 30–60 s per design

// ── Timing (ms) — defaults; overwritten from NVS at arm time ─────────────────
uint32_t STARTUP_DELAY_MS      = 10000;        // slept after arm, before 1st sample
uint32_t SERVO_OPEN_TIME_MS    = 5000;         // sample valve open duration
uint32_t INTER_SAMPLE_DELAY_MS = 64800000UL;   // 18 h (see decision #3)

// ── Hardware / calibration ────────────────────────────────────────────────────
#define MAIN_SERVO_CH    20
#define SERVO_FREQ       50
#define CLOSE_OFFSET_DEG 65
// !! MECHANICAL LIMIT: CLOSE_OFFSET_DEG MAX IS 65°. DO NOT INCREASE. !!
// !! RECALIBRATE THIS VALUE WHEN SWAPPING SERVOS.                     !!

const uint8_t CH_OPEN_DEG[21] = {
  100, 110, 100,  95,  95, 105,  95, 110,   // Ch0–7
   95, 105, 100, 105,  95, 115, 110,  95,   // Ch8–15
  100,  95,  95, 110,  90                   // Ch16–20 (Ch20 = main)
};

const uint8_t SERVO_CHANNELS[]   = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
const uint8_t SAMPLE_SERVO_COUNT = 20;
const uint16_t SERVO_SETTLE_MS   = 500;        // drive, then cut PWM after this

uint16_t chOpenPulse[21];
uint16_t chClosePulse[21];

// ── Persistent mission state ──────────────────────────────────────────────────
// NVS is authoritative (survives brownout / power loss). RTC memory is a fast
// mirror for the deep-sleep resume path and a cross-check (roadmap item).
#define STATE_MAGIC 0xC0FFEE01
RTC_DATA_ATTR uint32_t rtcMagic          = 0;
RTC_DATA_ATTR uint8_t  rtcNextSampleIdx  = 0;
RTC_DATA_ATTR uint8_t  rtcPhase          = 0;
RTC_DATA_ATTR bool     rtcMissionDone    = false;

enum Phase { PHASE_WARMUP = 0, PHASE_SAMPLING = 1 };

struct MissionState {
  bool     armed;
  uint8_t  nextSampleIdx;
  uint8_t  phase;
  bool     missionComplete;
  uint32_t startupMs, openMs, interMs;
};
MissionState st;
Preferences prefs;

Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);
WebServer server(80);

enum ServoPos { POS_CLOSE, POS_MID, POS_OPEN };
ServoPos servoPositions[21];
bool     servoHasPWM[21];

// ARM MODE runtime flags (only used while WiFi UI is up)
bool     armPending   = false;
uint32_t armStartMs   = 0;
bool     testMode     = false;   // bench bring-up: enables per-servo test tools
                                 // and locks out ARM/DEPLOY until exited

// ── Servo primitives ──────────────────────────────────────────────────────────
void setPWM(uint8_t ch, uint16_t pulse) {
  if (ch < 16) pwm0.setPWM(ch, 0, pulse);
  else         pwm1.setPWM(ch - 16, 0, pulse);
}

// Cut PWM — PCA9685 full-OFF forces the output LOW; the servo goes limp and
// holds its last position passively (zero holding current). See decision #1.
void releaseServo(uint8_t ch) {
  if (ch < 16) pwm0.setPWM(ch, 0, 4096);
  else         pwm1.setPWM(ch - 16, 0, 4096);
  servoHasPWM[ch] = false;
  Serial.printf("[REL]   Ch%2d → PWM off\n", ch);
}

void releaseAll() { for (uint8_t ch = 0; ch <= 20; ch++) releaseServo(ch); }

void openServo(uint8_t ch) {
  setPWM(ch, chOpenPulse[ch]);
  servoPositions[ch] = POS_OPEN;
  servoHasPWM[ch]    = true;
  Serial.printf("[OPEN]  Ch%2d → %3d° (pulse %d)\n", ch, CH_OPEN_DEG[ch], chOpenPulse[ch]);
}

void closeServo(uint8_t ch) {
  setPWM(ch, chClosePulse[ch]);
  servoPositions[ch] = POS_CLOSE;
  servoHasPWM[ch]    = true;
  uint8_t closeDeg = min((int)CH_OPEN_DEG[ch] + CLOSE_OFFSET_DEG, 180);
  Serial.printf("[CLOSE] Ch%2d → %3d° (pulse %d)\n", ch, closeDeg, chClosePulse[ch]);
}

void midServo(uint8_t ch) {
  setPWM(ch, 307);
  servoPositions[ch] = POS_MID;
  servoHasPWM[ch]    = true;
  Serial.printf("[MID]   Ch%2d → 90° (pulse 307)\n", ch);
}

// Move one channel to an arbitrary angle (bench testing only). Uses the SAME
// degree→count mapping as buildPulseTables(), so what you command here matches
// what the mission commands. Raw 0–180° ignores the 65° close limit — use it to
// check horn clocking, not to park a valve against its stop.
void angleServo(uint8_t ch, uint8_t deg) {
  uint16_t pulse = map(deg, 0, 180, 102, 512);
  setPWM(ch, pulse);
  servoPositions[ch] = POS_MID;
  servoHasPWM[ch]    = true;
  Serial.printf("[TEST]  Ch%2d → %3d° (pulse %d)\n", ch, deg, pulse);
}

// Sweep one channel across its CALIBRATED open↔close travel, then go limp.
// Stays inside the valve's real range so it can't over-rotate the pinch.
void sweepServo(uint8_t ch) {
  uint16_t lo = min(chOpenPulse[ch], chClosePulse[ch]);
  uint16_t hi = max(chOpenPulse[ch], chClosePulse[ch]);
  for (uint16_t p = lo; p <= hi; p += 6) { setPWM(ch, p); delay(15); }
  delay(150);
  for (int p = hi; p >= (int)lo; p -= 6) { setPWM(ch, (uint16_t)p); delay(15); }
  delay(150);
  releaseServo(ch);
}

uint8_t activePWMCount() {
  uint8_t n = 0;
  for (uint8_t ch = 0; ch <= 20; ch++) if (servoHasPWM[ch]) n++;
  return n;
}

// ── Hardware init helpers ─────────────────────────────────────────────────────
void buildPulseTables() {
  for (uint8_t i = 0; i <= 20; i++) {
    chOpenPulse[i]   = map(CH_OPEN_DEG[i], 0, 180, 102, 512);
    uint8_t closeDeg = min((int)CH_OPEN_DEG[i] + CLOSE_OFFSET_DEG, 180);
    chClosePulse[i]  = map(closeDeg, 0, 180, 102, 512);
    servoPositions[i] = POS_CLOSE;
  }
}

void initServoDrivers() {
  Wire.begin(21, 22);
  pwm0.begin(); pwm0.reset(); delay(10);
  pwm0.setOscillatorFrequency(25000000); pwm0.setPWMFreq(SERVO_FREQ);
  pwm1.begin(); pwm1.reset(); delay(10);
  pwm1.setOscillatorFrequency(25000000); pwm1.setPWMFreq(SERVO_FREQ);
  delay(10);
}

// Drop both PCA9685s to their low-power SLEEP mode (oscillator off, registers
// retained, outputs LOW). Cuts ~20 mA of chip idle to microamps.
void sleepServoDrivers() { pwm0.sleep(); pwm1.sleep(); }

void initBatteryAdc() {
  analogSetPinAttenuation(VBATT_SENSE_PIN, ADC_11db);   // 0–~3.1 V range
}

float readBatteryVoltage() {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < VBATT_SAMPLES; i++) { acc += analogReadMilliVolts(VBATT_SENSE_PIN); delay(2); }
  float vadc = (acc / (float)VBATT_SAMPLES) / 1000.0f;
  return vadc / VBATT_DIVIDER_RATIO;
}

// ── Mission-level valve actions ───────────────────────────────────────────────
void parkAllClosed() {                     // staggered close → release, inrush-limited
  Serial.println("[VALVE] park: staggered close + release");
  for (uint8_t ch = 0; ch <= 20; ch++) { closeServo(ch); delay(50); }
  delay(200);
  releaseAll();
}

void openMainAndHold() {                   // open main, then let it hold passively
  openServo(MAIN_SERVO_CH); delay(SERVO_SETTLE_MS); releaseServo(MAIN_SERVO_CH);
}

void releaseAllSamples() { for (uint8_t i = 0; i < SAMPLE_SERVO_COUNT; i++) releaseServo(SERVO_CHANNELS[i]); }

void takeSample(uint8_t idx) {
  uint8_t ch = SERVO_CHANNELS[idx];
  // isolate manifold
  closeServo(MAIN_SERVO_CH); delay(SERVO_SETTLE_MS); releaseServo(MAIN_SERVO_CH);
  // admit sample, hold open passively for the remainder of the open window
  openServo(ch); delay(SERVO_SETTLE_MS); releaseServo(ch);
  uint32_t hold = (SERVO_OPEN_TIME_MS > SERVO_SETTLE_MS) ? SERVO_OPEN_TIME_MS - SERVO_SETTLE_MS : 0;
  delay(hold);
  // seal sample
  closeServo(ch); delay(SERVO_SETTLE_MS); releaseServo(ch);
}

// ── Persistence ───────────────────────────────────────────────────────────────
void saveState() {
  prefs.begin("karen", false);
  prefs.putBool ("armed",   st.armed);
  prefs.putUChar("idx",     st.nextSampleIdx);
  prefs.putUChar("phase",   st.phase);
  prefs.putBool ("done",    st.missionComplete);
  prefs.putUInt ("startup", st.startupMs);
  prefs.putUInt ("open",    st.openMs);
  prefs.putUInt ("inter",   st.interMs);
  prefs.end();
  rtcMagic         = STATE_MAGIC;   // mirror to RTC
  rtcNextSampleIdx = st.nextSampleIdx;
  rtcPhase         = st.phase;
  rtcMissionDone   = st.missionComplete;
}

void loadState() {
  prefs.begin("karen", true);
  st.armed           = prefs.getBool ("armed",   false);
  st.nextSampleIdx   = prefs.getUChar("idx",     0);
  st.phase           = prefs.getUChar("phase",   PHASE_WARMUP);
  st.missionComplete = prefs.getBool ("done",    false);
  st.startupMs       = prefs.getUInt ("startup", STARTUP_DELAY_MS);
  st.openMs          = prefs.getUInt ("open",    SERVO_OPEN_TIME_MS);
  st.interMs         = prefs.getUInt ("inter",   INTER_SAMPLE_DELAY_MS);
  prefs.end();

  if (rtcMagic == STATE_MAGIC && rtcNextSampleIdx != st.nextSampleIdx)
    Serial.printf("[STATE] RTC idx %u != NVS idx %u — trusting NVS\n",
                  rtcNextSampleIdx, st.nextSampleIdx);

  STARTUP_DELAY_MS      = st.startupMs;
  SERVO_OPEN_TIME_MS    = st.openMs;
  INTER_SAMPLE_DELAY_MS = st.interMs;
}

// ── Deep sleep ────────────────────────────────────────────────────────────────
void radiosOff() { WiFi.mode(WIFI_OFF); /* BT never started */ }

void enterDeepSleep(uint64_t us) {
  digitalWrite(ARM_LED_PIN, LOW);
  sleepServoDrivers();
#ifdef SERVO_RAIL_EN_PIN
  digitalWrite(SERVO_RAIL_EN_PIN, SERVO_RAIL_EN_ACTIVE_HIGH ? LOW : HIGH);   // rail OFF
  gpio_hold_en((gpio_num_t)SERVO_RAIL_EN_PIN);   // latch through deep sleep
  gpio_deep_sleep_hold_en();
#endif
  Serial.printf("[SLEEP] deep sleep %llu s\n", us / 1000000ULL);
  Serial.flush();
  esp_sleep_enable_timer_wakeup(us);
  esp_deep_sleep_start();
}

// ── Mission wake (runs on every deep-sleep wake / brownout resume) ────────────
void runMissionWake(esp_sleep_wakeup_cause_t cause) {
  radiosOff();
#ifdef SERVO_RAIL_EN_PIN
  gpio_hold_dis((gpio_num_t)SERVO_RAIL_EN_PIN);
  pinMode(SERVO_RAIL_EN_PIN, OUTPUT);
  digitalWrite(SERVO_RAIL_EN_PIN, SERVO_RAIL_EN_ACTIVE_HIGH ? HIGH : LOW);   // rail ON
  delay(20);
#endif
  buildPulseTables();
  initBatteryAdc();
  initServoDrivers();

  float vb = readBatteryVoltage();
  Serial.printf("[WAKE]  cause=%d idx=%u phase=%u Vbatt=%.2f V\n",
                cause, st.nextSampleIdx, st.phase, vb);

  // Low-battery guard (decision #4): don't actuate on a weak pack.
  if (vb >= VBATT_PLAUSIBLE_MIN && vb < VBATT_CUTOFF_V) {
    Serial.println("[WAKE]  LOW BATTERY → park, no actuation");
    parkAllClosed();
    enterDeepSleep((uint64_t)INTER_SAMPLE_DELAY_MS * 1000ULL);
  }

  // Unexpected reset while armed (brownout / manual reset). See decision #5.
  if (cause != ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("[WAKE]  unexpected reset mid-mission → re-establish state");
    for (uint8_t i = 0; i < SAMPLE_SERVO_COUNT; i++) closeServo(SERVO_CHANNELS[i]);
    delay(200);
    releaseAllSamples();
    if (st.phase == PHASE_WARMUP) {
      enterDeepSleep((uint64_t)STARTUP_DELAY_MS * 1000ULL);
    } else {
      openMainAndHold();                                   // resume flushing
      enterDeepSleep((uint64_t)INTER_SAMPLE_DELAY_MS * 1000ULL);
    }
  }

  // Normal timer wake.
  if (st.phase == PHASE_WARMUP) {
    Serial.println("[WAKE]  warmup done → open main, begin flush");
    openMainAndHold();
    st.phase = PHASE_SAMPLING;
    saveState();
    enterDeepSleep((uint64_t)INTER_SAMPLE_DELAY_MS * 1000ULL);
  } else {  // PHASE_SAMPLING
    Serial.printf("[WAKE]  sample %u/%u (Ch%u)\n",
                  st.nextSampleIdx + 1, SAMPLE_SERVO_COUNT, SERVO_CHANNELS[st.nextSampleIdx]);
    takeSample(st.nextSampleIdx);
    st.nextSampleIdx++;
    if (st.nextSampleIdx >= SAMPLE_SERVO_COUNT) {
      st.missionComplete = true;
      saveState();
      Serial.println("[WAKE]  ALL SAMPLES COLLECTED → park, quiet sleep");
      parkAllClosed();
      enterDeepSleep((uint64_t)INTER_SAMPLE_DELAY_MS * 1000ULL);  // stays parked
    } else {
      openMainAndHold();                                   // reopen for next interval
      saveState();
      enterDeepSleep((uint64_t)INTER_SAMPLE_DELAY_MS * 1000ULL);
    }
  }

  enterDeepSleep((uint64_t)INTER_SAMPLE_DELAY_MS * 1000ULL);  // defensive catch-all
}

// ── Web UI (ARM MODE / bench only) ────────────────────────────────────────────
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Lander — Arm</title>
<style>
  :root{--bg:#0d1117;--card:#161b22;--border:#30363d;--text:#c9d1d9;--dim:#8b949e;
        --blue:#58a6ff;--green:#238636;--red:#da3633;--orange:#f0883e;--yellow:#9e6a03;--grey:#30363d;}
  *{box-sizing:border-box;}
  body{font-family:monospace;background:var(--bg);color:var(--text);padding:16px;max-width:620px;margin:0 auto;}
  h2{color:var(--blue);font-size:13px;text-transform:uppercase;letter-spacing:.08em;margin:0 0 8px;}
  .card{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:14px;margin-bottom:12px;}
  .status{text-align:center;}
  .state{font-size:18px;font-weight:bold;color:var(--blue);}
  .batt{font-size:34px;font-weight:bold;color:var(--orange);line-height:1.1;margin-top:4px;}
  .prog{font-size:13px;color:var(--dim);margin-top:3px;}
  .row{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:10px;}
  button{flex:1;min-width:60px;padding:9px 6px;border:none;border-radius:5px;font-size:13px;font-weight:bold;cursor:pointer;}
  .g{background:var(--green);color:#fff;} .r{background:var(--red);color:#fff;}
  .b{background:#1f6feb;color:#fff;} .gr{background:var(--grey);color:#fff;}
  .servo-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:5px;margin-bottom:10px;}
  .sc{background:var(--card);border:1px solid var(--border);border-radius:5px;padding:6px;text-align:center;}
  .sc.main-ch{border-color:#58a6ff55;} .sc.open{border-color:var(--green);background:#0d1f0f;}
  .sc.closed{border-color:var(--red);background:#1f0d0d;}
  .sc-name{font-size:11px;font-weight:bold;margin-bottom:1px;}
  .sc-pos{font-size:10px;color:var(--dim);margin-bottom:4px;}
  .sc.open .sc-pos{color:#3fb950;} .sc.closed .sc-pos{color:#f85149;}
  .sc button{min-width:unset;padding:4px 2px;font-size:11px;flex:1;}
  label{font-size:11px;color:var(--dim);display:block;margin-bottom:3px;}
  input[type=number]{background:var(--card);color:var(--text);border:1px solid var(--border);
    border-radius:4px;padding:6px 8px;width:100%;font-size:13px;}
  input.dirty{border-color:var(--orange);}
  .grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px;}
  .hms-row{display:flex;gap:4px;} .hms-row input{text-align:center;}
  .hms-label{display:flex;justify-content:space-between;align-items:baseline;}
  .hms-hint{font-size:10px;color:#555;}
  .note{font-size:11px;color:var(--dim);margin-bottom:8px;}
  .warn{font-size:11px;color:var(--orange);margin:6px 0 10px;}
  .sep{border:none;border-top:1px solid #21262d;margin:12px 0;}
  .apply-row{display:flex;gap:8px;align-items:center;} .apply-row button{flex:1;padding:10px;}
  #apply-status,#arm-status{font-size:11px;color:var(--dim);}
  .arm-btn{padding:14px;font-size:15px;}
</style>
</head>
<body>

<div class="card status">
  <div class="state" id="st">--</div>
  <div class="batt"  id="vb">-- V</div>
  <div class="prog"  id="pg"></div>
</div>

<h2>Servos</h2>
<div class="note" id="offset-note"></div>
<div class="servo-grid" id="grid"></div>
<div class="row">
  <button class="g"  onclick="cmd('cal_all_open')">All Open</button>
  <button class="r"  onclick="cmd('cal_all_close')">All Close</button>
  <button class="gr" onclick="cmd('cal_all_mid')">All 90°</button>
  <button class="gr" onclick="cmd('cal_all_release')">Release All</button>
</div>

<hr class="sep">
<h2>Timing</h2>
<div class="grid2" style="margin-bottom:10px">
  <div><label>Startup delay (ms)</label><input type="number" id="t_start" step="500" min="0"></div>
  <div><label>Sample open time (ms)</label><input type="number" id="t_open" step="100" min="100"></div>
</div>
<div style="margin-bottom:10px">
  <div class="hms-label"><label>Inter-sample interval</label>
    <span class="hms-hint">h : mm : ss &nbsp;·&nbsp; 18 hr = 18 : 00 : 00</span></div>
  <div class="hms-row">
    <input type="number" id="t_h"   placeholder="h"  min="0" max="99">
    <input type="number" id="t_m"   placeholder="mm" min="0" max="59">
    <input type="number" id="t_sec" placeholder="ss" min="0" max="59">
  </div>
</div>
<div class="apply-row">
  <button class="b" onclick="applySettings()">Apply Timing</button>
  <span id="apply-status"></span>
</div>

<hr class="sep">
<h2>Testing Mode</h2>
<div class="note">Bench bring-up — exercise one servo at a time as you wire it. Arming is locked while this is on; exit to arm.</div>
<div class="row">
  <button class="b" id="tm-btn" onclick="toggleTest()">Enter Testing Mode</button>
</div>
<div id="tm-panel" style="display:none">
  <div class="grid2" style="margin-bottom:10px">
    <div><label>Channel under test</label>
      <select id="tm-ch" style="background:var(--card);color:var(--text);border:1px solid var(--border);border-radius:4px;padding:6px 8px;width:100%;font-size:13px;"></select></div>
    <div><label>Raw angle · 0–180° · ignores 65° close limit</label>
      <div class="hms-row">
        <input type="range"  id="tm-deg"  min="0" max="180" value="90" style="flex:2"
               oninput="document.getElementById('tm-degv').value=this.value">
        <input type="number" id="tm-degv" min="0" max="180" value="90" style="max-width:60px"
               oninput="document.getElementById('tm-deg').value=this.value">
        <button class="gr" style="flex:1" onclick="tmove()">Move</button>
      </div></div>
  </div>
  <div class="row">
    <button class="g"  onclick="tsc('open')">Open</button>
    <button class="r"  onclick="tsc('close')">Close</button>
    <button class="gr" onclick="tsc('mid')">90°</button>
    <button class="b"  onclick="tsc('sweep')">Sweep</button>
    <button class="gr" onclick="tsc('release')">Release</button>
  </div>
  <div class="row">
    <button class="gr" onclick="thold('close')">Hold-check · closed</button>
    <button class="gr" onclick="thold('open')">Hold-check · open</button>
  </div>
  <div class="warn">Hold-check drives the valve to position, cuts PWM, and leaves it limp — then watch whether it creeps. This is the passive-hold test the whole deep-sleep budget depends on.</div>
</div>

<hr class="sep">
<h2>Arm / Deploy</h2>
<div class="warn">Arming closes all valves, blinks the ARM LED ~45 s (seal the
  enclosure now), then drops WiFi and begins the deep-sleep mission. There is no
  remote control once armed.</div>
<div class="row">
  <button class="g arm-btn" onclick="arm()">ARM &amp; DEPLOY</button>
  <button class="gr arm-btn" onclick="clearMission()">Clear / Disarm</button>
</div>
<div id="arm-status"></div>

<script>
const CHANNELS=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20];
const grid=document.getElementById('grid');
CHANNELS.forEach(ch=>{const m=ch===20;grid.innerHTML+=`
  <div class="sc${m?' main-ch':''}" id="sc${ch}">
    <div class="sc-name">${m?'MAIN':'Ch'+ch}</div>
    <div class="sc-pos" id="sp${ch}">--</div>
    <div class="row" style="margin:0">
      <button class="g" onclick="sc(${ch},'open')">O</button>
      <button class="r" onclick="sc(${ch},'close')">C</button>
    </div>
  </div>`;});

function cmd(a){fetch('/control?action='+a).catch(()=>{});}
function sc(ch,a){fetch('/servo?ch='+ch+'&action='+a).catch(()=>{});}

// Testing mode
const tsel=document.getElementById('tm-ch');
CHANNELS.forEach(ch=>{const o=document.createElement('option');
  o.value=ch;o.textContent=(ch===20?'MAIN (Ch20)':'Ch'+ch);tsel.appendChild(o);});
function tch(){return parseInt(document.getElementById('tm-ch').value);}
function tsc(a){fetch('/servo?ch='+tch()+'&action='+a).catch(()=>{});}
function tmove(){fetch('/servo?ch='+tch()+'&action=angle&deg='+document.getElementById('tm-degv').value).catch(()=>{});}
function thold(pos){fetch('/servo?ch='+tch()+'&action=hold&pos='+pos).catch(()=>{});}
function toggleTest(){const on=document.getElementById('tm-panel').style.display==='none';
  fetch('/testmode?on='+(on?1:0)).catch(()=>{});}

let timingDirty=false;
const timingIds=['t_start','t_open','t_h','t_m','t_sec'];
timingIds.forEach(id=>{document.getElementById(id).addEventListener('input',()=>{
  timingDirty=true;
  timingIds.forEach(i=>document.getElementById(i).classList.add('dirty'));
  document.getElementById('apply-status').textContent='unsaved';
  document.getElementById('apply-status').style.color='#f0883e';});});

function applySettings(){
  const h=parseInt(document.getElementById('t_h').value)||0;
  const m=parseInt(document.getElementById('t_m').value)||0;
  const s=parseInt(document.getElementById('t_sec').value)||0;
  const interMs=(h*3600+m*60+s)*1000;
  const start=document.getElementById('t_start').value;
  const open=document.getElementById('t_open').value;
  fetch(`/settings?startup=${start}&open=${open}&inter=${interMs}`).then(()=>{
    timingDirty=false;
    timingIds.forEach(i=>document.getElementById(i).classList.remove('dirty'));
    document.getElementById('apply-status').textContent='saved ✓';
    document.getElementById('apply-status').style.color='#3fb950';
    setTimeout(()=>{document.getElementById('apply-status').textContent='';},2000);
  }).catch(()=>{document.getElementById('apply-status').textContent='send failed';
    document.getElementById('apply-status').style.color='#f85149';});
}

function arm(){
  if(!confirm('Arm and deploy? WiFi will drop after the ~45 s LED blink and the '
    +'mission will run on deep sleep with no remote control. Seal the enclosure '
    +'during the blink.')) return;
  fetch('/arm?confirm=1').then(()=>{
    document.getElementById('arm-status').textContent='ARMED — LED blinking, seal now. WiFi will drop.';
    document.getElementById('arm-status').style.color='#f0883e';
  }).catch(()=>{document.getElementById('arm-status').textContent='send failed';});
}
function clearMission(){
  if(!confirm('Clear mission state (disarm, reset sample index to 0)?')) return;
  fetch('/clear?confirm=1').then(()=>{
    document.getElementById('arm-status').textContent='Cleared / disarmed.';
    document.getElementById('arm-status').style.color='#3fb950';
  }).catch(()=>{});
}

function msToHMS(ms){const t=Math.floor(ms/1000);return{h:Math.floor(t/3600),m:Math.floor((t%3600)/60),s:t%60};}

function poll(){
  fetch('/status').then(r=>r.json()).then(d=>{
    document.getElementById('st').textContent=d.state;
    document.getElementById('vb').textContent=(d.vbatt?d.vbatt.toFixed(2):'--')+' V';
    document.getElementById('vb').style.color=(d.vbatt&&d.vbatt<5.6)?'#f85149':'#f0883e';
    document.getElementById('pg').textContent=
      d.armed?('Armed · next sample '+(parseInt(d.idx)+1)+' of '+d.total)
             :(d.complete?'Mission complete · '+d.total+' samples':'Not armed');
    const test=!!d.test;
    document.getElementById('tm-panel').style.display=test?'block':'none';
    const tb=document.getElementById('tm-btn');
    tb.textContent=test?'Exit Testing Mode':'Enter Testing Mode';
    tb.className=test?'r':'b';
    const ab=document.querySelector('.arm-btn.g');
    if(ab){ab.disabled=test;ab.style.opacity=test?'0.4':'1';ab.style.cursor=test?'not-allowed':'pointer';}
    if(!timingDirty){
      if(d.startup_ms!==undefined)document.getElementById('t_start').value=d.startup_ms;
      if(d.open_ms!==undefined)document.getElementById('t_open').value=d.open_ms;
      if(d.inter_ms!==undefined){const t=msToHMS(parseInt(d.inter_ms));
        document.getElementById('t_h').value=t.h;document.getElementById('t_m').value=t.m;
        document.getElementById('t_sec').value=t.s;}
    }
    if(d.close_offset)document.getElementById('offset-note').textContent=
      'Open = calibrated · Close = open +'+d.close_offset+'° toward 180°';
    CHANNELS.forEach(ch=>{const c=document.getElementById('sc'+ch),p=document.getElementById('sp'+ch);
      const s=d.servoStates[ch];
      c.className='sc'+(ch===20?' main-ch':'')+(s==='open'?' open':s==='close'?' closed':'');
      p.textContent=s==='open'?'OPEN':s==='close'?'CLOSED':'MID';});
  }).catch(()=>{});
}
setInterval(poll,750); poll();
</script>
</body>
</html>
)rawliteral";

// ── Web handlers ──────────────────────────────────────────────────────────────
void handleRoot() { server.send(200, "text/html", HTML_PAGE); }

void handleStatus() {
  const char* stateStr = st.missionComplete ? "COMPLETE"
                       : testMode           ? "TESTING"
                       : (st.armed ? "ARMED" : "ARM MODE");
  String json = "{";
  json += "\"state\":\""     + String(stateStr)             + "\",";
  json += "\"armed\":"       + String(st.armed ? 1 : 0)     + ",";
  json += "\"complete\":"    + String(st.missionComplete?1:0)+ ",";
  json += "\"idx\":"         + String(st.nextSampleIdx)     + ",";
  json += "\"total\":"       + String(SAMPLE_SERVO_COUNT)   + ",";
  json += "\"startup_ms\":"  + String(STARTUP_DELAY_MS)     + ",";
  json += "\"open_ms\":"     + String(SERVO_OPEN_TIME_MS)   + ",";
  json += "\"inter_ms\":"    + String(INTER_SAMPLE_DELAY_MS)+ ",";
  json += "\"close_offset\":"+ String(CLOSE_OFFSET_DEG)     + ",";
  json += "\"test\":"        + String(testMode ? 1 : 0)     + ",";
  json += "\"vbatt\":"       + String(readBatteryVoltage(),2)+ ",";
  json += "\"servoStates\":[";
  for (int i = 0; i <= 20; i++) {
    switch (servoPositions[i]) {
      case POS_OPEN: json += "\"open\"";  break;
      case POS_MID:  json += "\"mid\"";   break;
      default:       json += "\"close\""; break;
    }
    if (i < 20) json += ",";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleControl() {
  if (!server.hasArg("action")) { server.send(400, "text/plain", "Missing action"); return; }
  String a = server.arg("action");
  if      (a == "cal_all_open")    { for (uint8_t ch = 0; ch <= 20; ch++) openServo(ch); }
  else if (a == "cal_all_close")   { for (uint8_t ch = 0; ch <= 20; ch++) closeServo(ch); }
  else if (a == "cal_all_mid")     { for (uint8_t ch = 0; ch <= 20; ch++) midServo(ch); }
  else if (a == "cal_all_release") { releaseAll(); }
  server.send(200, "text/plain", "OK");
}

void handleServo() {
  if (!server.hasArg("ch") || !server.hasArg("action")) { server.send(400,"text/plain","Missing args"); return; }
  uint8_t ch = (uint8_t)server.arg("ch").toInt();
  String  a  = server.arg("action");
  if (ch > 20) { server.send(400,"text/plain","Bad channel"); return; }
  if      (a == "open")    openServo(ch);
  else if (a == "close")   closeServo(ch);
  else if (a == "mid")     midServo(ch);
  else if (a == "release") releaseServo(ch);
  else if (a == "angle" || a == "sweep" || a == "hold") {
    if (!testMode) { server.send(409, "text/plain", "testing mode is off"); return; }
    if (a == "angle") {
      int d = server.hasArg("deg") ? server.arg("deg").toInt() : 90;
      angleServo(ch, (uint8_t)constrain(d, 0, 180));
    } else if (a == "sweep") {
      sweepServo(ch);
    } else {  // hold-check: drive to position, cut PWM, leave limp to watch creep
      if (server.arg("pos") == "open") openServo(ch); else closeServo(ch);
      delay(SERVO_SETTLE_MS);
      releaseServo(ch);
    }
  }
  server.send(200,"text/plain","OK");
}

void handleSettings() {
  if (server.hasArg("startup")) STARTUP_DELAY_MS      = server.arg("startup").toInt();
  if (server.hasArg("open"))    SERVO_OPEN_TIME_MS    = server.arg("open").toInt();
  if (server.hasArg("inter"))   INTER_SAMPLE_DELAY_MS = server.arg("inter").toInt();
  Serial.printf("[WEB]   timing startup:%lu open:%lu inter:%lu ms\n",
                STARTUP_DELAY_MS, SERVO_OPEN_TIME_MS, INTER_SAMPLE_DELAY_MS);
  server.send(200,"text/plain","OK");
}

void handleArm() {
  if (testMode) { server.send(409,"text/plain","exit testing mode before arming"); return; }
  if (server.arg("confirm") != "1") { server.send(400,"text/plain","confirm=1 required"); return; }
  st.armed           = true;
  st.nextSampleIdx   = 0;
  st.phase           = PHASE_WARMUP;
  st.missionComplete = false;
  st.startupMs       = STARTUP_DELAY_MS;
  st.openMs          = SERVO_OPEN_TIME_MS;
  st.interMs         = INTER_SAMPLE_DELAY_MS;
  saveState();
  armPending = true;
  armStartMs = millis();
  Serial.println("[ARM]   armed; blinking LED then deep sleep");
  server.send(200,"text/plain","ARMED");
}

void handleClear() {
  if (server.arg("confirm") != "1") { server.send(400,"text/plain","confirm=1 required"); return; }
  st.armed = false; st.nextSampleIdx = 0; st.phase = PHASE_WARMUP; st.missionComplete = false;
  saveState();
  armPending = false;
  digitalWrite(ARM_LED_PIN, LOW);
  Serial.println("[ARM]   mission cleared / disarmed");
  server.send(200,"text/plain","CLEARED");
}

void handleTestMode() {
  if (armPending) { server.send(409,"text/plain","arming in progress"); return; }
  bool on = (server.arg("on") == "1");
  testMode = on;
  if (!on) releaseAll();          // leave everything limp/safe when leaving test mode
  Serial.printf("[TEST]  testing mode %s\n", on ? "ON" : "OFF");
  server.send(200,"text/plain", on ? "TEST_ON" : "TEST_OFF");
}

void handleNotFound() { server.send(404,"text/plain","Not found"); }

// ── ARM MODE (WiFi UI up; bench + pre-seal arming) ────────────────────────────
void startArmMode() {
  buildPulseTables();
  initBatteryAdc();
  initServoDrivers();
  parkAllClosed();                          // known starting state

  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("[AP]    %s  http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  server.on("/",         handleRoot);
  server.on("/status",   handleStatus);
  server.on("/control",  handleControl);
  server.on("/servo",    handleServo);
  server.on("/testmode", handleTestMode);
  server.on("/settings", handleSettings);
  server.on("/arm",      handleArm);
  server.on("/clear",    handleClear);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("[AP]    ready — calibrate, set timing, then ARM");
}

// ── Setup / Loop ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);
  memset(servoHasPWM, false, sizeof(servoHasPWM));

  pinMode(ARM_LED_PIN, OUTPUT);
  digitalWrite(ARM_LED_PIN, LOW);
#ifdef SERVO_RAIL_EN_PIN
  gpio_hold_dis((gpio_num_t)SERVO_RAIL_EN_PIN);
  pinMode(SERVO_RAIL_EN_PIN, OUTPUT);
#endif

  loadState();
  esp_reset_reason_t     rr = esp_reset_reason();
  esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
  Serial.printf("=== BOOT reset=%d wake=%d armed=%d done=%d idx=%u ===\n",
                rr, wc, st.armed, st.missionComplete, st.nextSampleIdx);

  bool selfWoke = (rr == ESP_RST_DEEPSLEEP);
  bool brownout = (rr == ESP_RST_BROWNOUT);

  if (st.armed && !st.missionComplete && (selfWoke || brownout)) {
    runMissionWake(wc);                     // never returns — ends in deep sleep
  }
  if (st.missionComplete && (selfWoke || brownout)) {
    Serial.println("[BOOT]  mission complete — park + quiet sleep");
    buildPulseTables(); initServoDrivers(); parkAllClosed();
    enterDeepSleep((uint64_t)INTER_SAMPLE_DELAY_MS * 1000ULL);
  }

  // Fresh power-on / EN reset (operator on the bench) → ARM MODE.
  startArmMode();
}

void loop() {
  server.handleClient();

  if (armPending) {
    // Blink the ARM LED through the seal window, then commit and deep-sleep.
    digitalWrite(ARM_LED_PIN, (millis() / 250) % 2);
    if (millis() - armStartMs >= ARM_BLINK_MS) {
      armPending = false;
      Serial.println("[ARM]   seal window done → park + begin mission");
      parkAllClosed();                      // all closed for warmup
      enterDeepSleep((uint64_t)STARTUP_DELAY_MS * 1000ULL);
    }
  }
}