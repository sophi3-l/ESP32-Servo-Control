#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

// ── Network ────────────────────────────────────────────────────────────────
const char* AP_SSID     = "LanderController";
const char* AP_PASSWORD = "lander1234";

// ── Timing (all ms) ────────────────────────────────────────────────────────
uint32_t STARTUP_DELAY_MS      = 10000;   // warmup before first sample
uint32_t SERVO_OPEN_TIME_MS    = 5000;    // how long each sample servo stays open
uint32_t INTER_SAMPLE_DELAY_MS = 30000;   // wait between samples
                                          // DEPLOYMENT: 28800000  (8 hr)
                                          // BENCH TEST:    30000  (30 s)

const uint16_t SERVO_SETTLE_MS = 500;     // hold PWM after a move, then cut signal
bool REPEAT_SEQUENCE = false;

// ── Hardware / calibration ─────────────────────────────────────────────────
#define MAIN_SERVO_CH    20
#define SERVO_FREQ       50
#define CLOSE_OFFSET_DEG 65
// !! MECHANICAL LIMIT: CLOSE_OFFSET_DEG MAX IS 65°. DO NOT INCREASE. !!
// !! RECALIBRATE THIS VALUE WHEN SWAPPING SERVOS.                     !!

// Open-position angles for Ch0–20.
// Ch0–15 → board 0x40 ;  Ch16–20 → board 0x41 (local ch = ch − 16).
const uint8_t CH_OPEN_DEG[21] = {
  100, 110, 100,  95,  95, 105,  95, 110,   // Ch0–7
   95, 105, 100, 105,  95, 115, 110,  95,   // Ch8–15
  100,  95,  95, 110,  90                   // Ch16–20
};

// Sample servo channels only — Ch20 (main) is handled separately.
const uint8_t SERVO_CHANNELS[]   = {
  0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19
};
const uint8_t SAMPLE_SERVO_COUNT = 20;

uint16_t chOpenPulse[21];
uint16_t chClosePulse[21];

// ── State machine ──────────────────────────────────────────────────────────
//
//  Normal sequence:
//    STARTUP_COUNTDOWN → OPEN_MAIN → INTER_SAMPLE_WAIT
//                                         ↓
//                                    CLOSE_MAIN → SAMPLE_OPEN → CLOSE_SAMPLE
//                                         ↑___________________________↓
//                                    (repeat, or COMPLETE on last sample)
//
//  Stop: STOPPING → (settle) → releaseAll → MANUAL
//
enum SystemState {
  STATE_CALIBRATION,
  STATE_STARTUP_COUNTDOWN,
  STATE_STARTUP_PAUSED,
  STATE_OPEN_MAIN,
  STATE_INTER_SAMPLE_WAIT,
  STATE_CLOSE_MAIN,
  STATE_SAMPLE_OPEN,
  STATE_CLOSE_SAMPLE,
  STATE_STOPPING,
  STATE_COMPLETE,
  STATE_MANUAL
};

SystemState currentState     = STATE_CALIBRATION;
uint32_t    stateStartTime   = 0;
uint32_t    startupRemaining = 0;
uint8_t     currentServoIdx  = 0;
bool        sampleReleased   = false;

Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);
WebServer server(80);

enum ServoPos { POS_CLOSE, POS_MID, POS_OPEN };
ServoPos servoPositions[21];

// ── PWM activity tracking ──────────────────────────────────────────────────
//
// Tracks which channels are currently receiving a PWM signal from the PCA9685.
// Key for the Stop command: in normal operation at most 1–2 channels are active
// at any moment.  By only closing those channels (not all 21), Stop avoids the
// large inrush current spike that previously caused a PCA9685 brownout and
// corrupted the full-OFF register writes.
//
bool servoHasPWM[21];   // initialised to false in setup()

// ── Servo primitives ───────────────────────────────────────────────────────
void setPWM(uint8_t ch, uint16_t pulse) {
  if (ch < 16) pwm0.setPWM(ch, 0, pulse);
  else          pwm1.setPWM(ch - 16, 0, pulse);
}

// Cut PWM signal — PCA9685 full-OFF bit forces output LOW.
// Servo holds last physical position passively (zero holding current).
//
// NOTE: if a servo must actively resist external force (water pressure,
// spring tension), removing the signal will let it drift.  Comment out
// the releaseServo() call for that servo if needed.
void releaseServo(uint8_t ch) {
  if (ch < 16) pwm0.setPWM(ch, 0, 4096);
  else          pwm1.setPWM(ch - 16, 0, 4096);
  servoHasPWM[ch] = false;
  Serial.printf("[REL]   Ch%2d → PWM off\n", ch);
}

void releaseAll() {
  for (uint8_t ch = 0; ch <= 20; ch++) releaseServo(ch);
}

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

// ── Status helpers ─────────────────────────────────────────────────────────
String getStateString() {
  switch (currentState) {
    case STATE_CALIBRATION:       return "Calibration";
    case STATE_STARTUP_COUNTDOWN: return "Startup";
    case STATE_STARTUP_PAUSED:    return "Paused";
    case STATE_OPEN_MAIN:         return "Opening Main";
    case STATE_INTER_SAMPLE_WAIT: return "Idle";
    case STATE_CLOSE_MAIN:        return "Transitioning";
    case STATE_SAMPLE_OPEN:       return "Sampling";
    case STATE_CLOSE_SAMPLE:      return "Closing";
    case STATE_STOPPING:          return "Stopping";
    case STATE_COMPLETE:          return "Complete";
    case STATE_MANUAL:            return "Manual";
    default:                      return "Unknown";
  }
}

uint32_t getTimeRemaining() {
  uint32_t elapsed = millis() - stateStartTime;
  switch (currentState) {
    case STATE_STARTUP_COUNTDOWN:
      return STARTUP_DELAY_MS > elapsed ? STARTUP_DELAY_MS - elapsed : 0;
    case STATE_STARTUP_PAUSED:
      return startupRemaining;
    case STATE_INTER_SAMPLE_WAIT:
      return INTER_SAMPLE_DELAY_MS > elapsed ? INTER_SAMPLE_DELAY_MS - elapsed : 0;
    case STATE_SAMPLE_OPEN:
      return SERVO_OPEN_TIME_MS > elapsed ? SERVO_OPEN_TIME_MS - elapsed : 0;
    case STATE_STOPPING:
      return SERVO_SETTLE_MS > elapsed ? SERVO_SETTLE_MS - elapsed : 0;
    default: return 0;
  }
}

// Count how many channels currently have live PWM (useful for diagnostics).
uint8_t activePWMCount() {
  uint8_t n = 0;
  for (uint8_t ch = 0; ch <= 20; ch++) if (servoHasPWM[ch]) n++;
  return n;
}

// ── Web page ───────────────────────────────────────────────────────────────
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Lander</title>
<style>
  :root {
    --bg:#0d1117; --card:#161b22; --border:#30363d; --text:#c9d1d9;
    --dim:#8b949e; --blue:#58a6ff; --green:#238636; --red:#da3633;
    --orange:#f0883e; --yellow:#9e6a03; --grey:#30363d;
  }
  * { box-sizing:border-box; }
  body { font-family:monospace; background:var(--bg); color:var(--text);
         padding:16px; max-width:620px; margin:0 auto; }
  h2 { color:var(--blue); font-size:13px; text-transform:uppercase;
       letter-spacing:.08em; margin:0 0 8px; }
  .card { background:var(--card); border:1px solid var(--border);
          border-radius:8px; padding:14px; margin-bottom:12px; }
  .status { text-align:center; }
  .state  { font-size:15px; font-weight:bold; color:var(--blue); }
  .timer  { font-size:48px; font-weight:bold; color:var(--orange); line-height:1.05; }
  .prog   { font-size:13px; color:var(--dim); margin-top:3px; }
  .status-msg { font-size:11px; color:#6e7681; margin-top:2px; }
  .row    { display:flex; gap:6px; flex-wrap:wrap; margin-bottom:10px; }
  button  { flex:1; min-width:60px; padding:9px 6px; border:none; border-radius:5px;
            font-size:13px; font-weight:bold; cursor:pointer; }
  .g  { background:var(--green);  color:#fff; }
  .r  { background:var(--red);    color:#fff; }
  .b  { background:#1f6feb;       color:#fff; }
  .y  { background:var(--yellow); color:#fff; }
  .gr { background:var(--grey);   color:#fff; }
  .servo-grid { display:grid; grid-template-columns:repeat(4,1fr);
                gap:5px; margin-bottom:10px; }
  .sc { background:var(--card); border:1px solid var(--border);
        border-radius:5px; padding:6px; text-align:center; }
  .sc.main-ch { border-color:#58a6ff55; }
  .sc.open    { border-color:var(--green); background:#0d1f0f; }
  .sc.closed  { border-color:var(--red);   background:#1f0d0d; }
  .sc-name { font-size:11px; font-weight:bold; margin-bottom:1px; }
  .sc-pos  { font-size:10px; color:var(--dim); margin-bottom:4px; }
  .sc.open   .sc-pos { color:#3fb950; }
  .sc.closed .sc-pos { color:#f85149; }
  .sc button { min-width:unset; padding:4px 2px; font-size:11px; flex:1; }
  label { font-size:11px; color:var(--dim); display:block; margin-bottom:3px; }
  input[type=number] { background:var(--card); color:var(--text);
    border:1px solid var(--border); border-radius:4px;
    padding:6px 8px; width:100%; font-size:13px; }
  input.dirty { border-color:var(--orange); }
  .grid2 { display:grid; grid-template-columns:1fr 1fr; gap:8px; }
  .hms-row { display:flex; gap:4px; }
  .hms-row input { text-align:center; }
  .hms-label { display:flex; justify-content:space-between; align-items:baseline; }
  .hms-hint  { font-size:10px; color:#555; }
  .note { font-size:11px; color:var(--dim); margin-bottom:8px; }
  .sep  { border:none; border-top:1px solid #21262d; margin:12px 0; }
  .apply-row { display:flex; gap:8px; align-items:center; }
  .apply-row button { flex:1; padding:10px; }
  #apply-status { font-size:11px; color:var(--dim); }
</style>
</head>
<body>

<div class="card status">
  <div class="state"      id="st">--</div>
  <div class="timer"      id="tm">--</div>
  <div class="prog"       id="pg"></div>
  <div class="status-msg" id="sm"></div>
</div>

<h2>Sequence</h2>
<div class="row">
  <button class="y" onclick="cmd('pause')">Pause</button>
  <button class="g" onclick="cmd('resume')">Resume</button>
  <button class="b" onclick="cmd('start')">Start Now</button>
  <button class="b" onclick="cmd('next_sample')">Next Sample</button>
  <button class="r" onclick="cmd('stop')">Stop / Close All</button>
</div>

<hr class="sep">
<h2>Servos</h2>
<div class="note" id="offset-note"></div>
<div class="servo-grid" id="grid"></div>

<h2>All Servos</h2>
<div class="row">
  <button class="g"  onclick="cmd('cal_all_open')">All Open</button>
  <button class="r"  onclick="cmd('cal_all_close')">All Close</button>
  <button class="gr" onclick="cmd('cal_all_mid')">All 90°</button>
  <button class="gr" onclick="cmd('cal_all_release')">Release All</button>
</div>

<hr class="sep">
<h2>Timing</h2>
<div class="grid2" style="margin-bottom:10px">
  <div>
    <label>Startup delay (ms)</label>
    <input type="number" id="t_start" step="500" min="0">
  </div>
  <div>
    <label>Sample open time (ms)</label>
    <input type="number" id="t_open" step="100" min="100">
  </div>
</div>
<div style="margin-bottom:10px">
  <div class="hms-label">
    <label>Inter-sample delay</label>
    <span class="hms-hint">h : mm : ss &nbsp;·&nbsp; 8 hr = 8 : 00 : 00</span>
  </div>
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

<script>
const CHANNELS = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20];
const grid = document.getElementById('grid');
CHANNELS.forEach(ch => {
  const isMain = ch === 20;
  grid.innerHTML += `
    <div class="sc${isMain?' main-ch':''}" id="sc${ch}">
      <div class="sc-name">${isMain ? 'MAIN' : 'Ch' + ch}</div>
      <div class="sc-pos" id="sp${ch}">--</div>
      <div class="row" style="margin:0">
        <button class="g" onclick="sc(${ch},'open')">O</button>
        <button class="r" onclick="sc(${ch},'close')">C</button>
      </div>
    </div>`;
});

function cmd(a)   { fetch('/control?action='+a).catch(()=>{}); }
function sc(ch,a) { fetch('/servo?ch='+ch+'&action='+a).catch(()=>{}); }

// ── Timing dirty flag ─────────────────────────────────────────────────────
// Prevents poll() from overwriting inputs while the user is editing them.
let timingDirty = false;
const timingIds = ['t_start','t_open','t_h','t_m','t_sec'];
timingIds.forEach(id => {
  document.getElementById(id).addEventListener('input', () => {
    timingDirty = true;
    timingIds.forEach(i => document.getElementById(i).classList.add('dirty'));
    document.getElementById('apply-status').textContent = 'unsaved';
    document.getElementById('apply-status').style.color = '#f0883e';
  });
});

function applySettings() {
  const h       = parseInt(document.getElementById('t_h').value)   || 0;
  const m       = parseInt(document.getElementById('t_m').value)   || 0;
  const s       = parseInt(document.getElementById('t_sec').value) || 0;
  const interMs = (h * 3600 + m * 60 + s) * 1000;
  const start   = document.getElementById('t_start').value;
  const open    = document.getElementById('t_open').value;
  fetch(`/settings?startup=${start}&open=${open}&inter=${interMs}`)
    .then(() => {
      timingDirty = false;
      timingIds.forEach(i => document.getElementById(i).classList.remove('dirty'));
      document.getElementById('apply-status').textContent = 'saved ✓';
      document.getElementById('apply-status').style.color = '#3fb950';
      setTimeout(() => { document.getElementById('apply-status').textContent = ''; }, 2000);
    })
    .catch(() => {
      document.getElementById('apply-status').textContent = 'send failed';
      document.getElementById('apply-status').style.color = '#f85149';
    });
}

function formatTime(ms) {
  if (ms <= 0) return '--';
  const totalSec = Math.floor(ms / 1000);
  if (totalSec < 60) return (ms / 1000).toFixed(1) + 's';
  const h   = Math.floor(totalSec / 3600);
  const min = Math.floor((totalSec % 3600) / 60);
  const sec = totalSec % 60;
  const pad = n => String(n).padStart(2, '0');
  return h > 0 ? `${h}:${pad(min)}:${pad(sec)}` : `${min}:${pad(sec)}`;
}

function msToHMS(ms) {
  const t = Math.floor(ms / 1000);
  return { h:Math.floor(t/3600), m:Math.floor((t%3600)/60), s:t%60 };
}

function poll() {
  fetch('/status').then(r => r.json()).then(d => {
    document.getElementById('st').textContent = d.state;
    document.getElementById('sm').textContent = d.message;
    document.getElementById('tm').textContent = formatTime(parseInt(d.remaining));

    if (d.sample_total) {
      const idx = parseInt(d.sample_idx), tot = parseInt(d.sample_total);
      document.getElementById('pg').textContent =
        idx < tot ? `Sample ${idx+1} of ${tot}` : 'All samples collected';
    } else {
      document.getElementById('pg').textContent = '';
    }

    if (!timingDirty) {
      if (d.startup_ms !== undefined) document.getElementById('t_start').value = d.startup_ms;
      if (d.open_ms    !== undefined) document.getElementById('t_open').value  = d.open_ms;
      if (d.inter_ms   !== undefined) {
        const t = msToHMS(parseInt(d.inter_ms));
        document.getElementById('t_h').value   = t.h;
        document.getElementById('t_m').value   = t.m;
        document.getElementById('t_sec').value = t.s;
      }
    }

    if (d.close_offset) {
      document.getElementById('offset-note').textContent =
        'Open = calibrated · Close = open +' + d.close_offset + '° toward 180°';
    }

    CHANNELS.forEach(ch => {
      const card = document.getElementById('sc'+ch);
      const pos  = document.getElementById('sp'+ch);
      const st   = d.servoStates[ch];
      card.className = 'sc' + (ch===20?' main-ch':'') +
        (st==='open'?' open': st==='close'?' closed':'');
      pos.textContent = st==='open'?'OPEN': st==='close'?'CLOSED':'MID';
    });
  }).catch(()=>{});
}
setInterval(poll, 500);
poll();
</script>
</body>
</html>
)rawliteral";

// ── Web handlers ───────────────────────────────────────────────────────────
void handleRoot() { server.send(200, "text/html", HTML_PAGE); }

void handleStatus() {
  String msg;
  switch (currentState) {
    case STATE_CALIBRATION:         msg = "All servos open (calibration)"; break;
    case STATE_STARTUP_COUNTDOWN:
    case STATE_STARTUP_PAUSED:      msg = "Waiting to deploy..."; break;
    case STATE_OPEN_MAIN:           msg = "Main servo opening"; break;
    case STATE_INTER_SAMPLE_WAIT:
      msg = currentServoIdx < SAMPLE_SERVO_COUNT
          ? "Idle — next: Ch" + String(SERVO_CHANNELS[currentServoIdx])
          : "Waiting (all samples done)";
      break;
    case STATE_CLOSE_MAIN:    msg = "Main → Ch" + String(SERVO_CHANNELS[currentServoIdx]); break;
    case STATE_SAMPLE_OPEN:   msg = "Ch" + String(SERVO_CHANNELS[currentServoIdx]) + " collecting"; break;
    case STATE_CLOSE_SAMPLE:  msg = "Ch" + String(SERVO_CHANNELS[currentServoIdx]) + " closing"; break;
    case STATE_STOPPING:      msg = "Closing active channels..."; break;
    case STATE_COMPLETE:      msg = "All samples collected"; break;
    case STATE_MANUAL:        msg = "Manual control"; break;
    default:                  msg = "";
  }

  String json = "{";
  json += "\"state\":\""       + getStateString()              + "\",";
  json += "\"remaining\":"     + String(getTimeRemaining())    + ",";
  json += "\"message\":\""     + msg                           + "\",";
  json += "\"sample_idx\":"    + String(currentServoIdx)       + ",";
  json += "\"sample_total\":"  + String(SAMPLE_SERVO_COUNT)    + ",";
  json += "\"startup_ms\":"    + String(STARTUP_DELAY_MS)      + ",";
  json += "\"open_ms\":"       + String(SERVO_OPEN_TIME_MS)    + ",";
  json += "\"inter_ms\":"      + String(INTER_SAMPLE_DELAY_MS) + ",";
  json += "\"close_offset\":"  + String(CLOSE_OFFSET_DEG)      + ",";
  json += "\"servoStates\":[";
  for (int i = 0; i <= 20; i++) {
    switch (servoPositions[i]) {
      case POS_OPEN:  json += "\"open\"";  break;
      case POS_MID:   json += "\"mid\"";   break;
      default:        json += "\"close\""; break;
    }
    if (i < 20) json += ",";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleControl() {
  if (!server.hasArg("action")) { server.send(400,"text/plain","Missing action"); return; }
  String   action = server.arg("action");
  uint32_t now    = millis();

  if      (action == "cal_all_open")    { for (uint8_t ch=0; ch<=20; ch++) openServo(ch); }
  else if (action == "cal_all_close")   { for (uint8_t ch=0; ch<=20; ch++) closeServo(ch); }
  else if (action == "cal_all_mid")     { for (uint8_t ch=0; ch<=20; ch++) midServo(ch); }
  else if (action == "cal_all_release") { releaseAll(); }

  else if (action == "pause") {
    if (currentState == STATE_STARTUP_COUNTDOWN) {
      uint32_t elapsed = now - stateStartTime;
      startupRemaining = STARTUP_DELAY_MS > elapsed ? STARTUP_DELAY_MS - elapsed : 0;
      currentState = STATE_STARTUP_PAUSED;
    }
  }
  else if (action == "resume") {
    if (currentState == STATE_STARTUP_PAUSED) {
      stateStartTime = now - (STARTUP_DELAY_MS - startupRemaining);
      currentState   = STATE_STARTUP_COUNTDOWN;
    }
  }
  else if (action == "start") {
    currentServoIdx = 0;
    sampleReleased  = false;
    closeServo(MAIN_SERVO_CH);
    stateStartTime = now;
    currentState   = STATE_CLOSE_MAIN;
  }
  else if (action == "next_sample") {
    if (currentState == STATE_INTER_SAMPLE_WAIT)
      stateStartTime = now - INTER_SAMPLE_DELAY_MS;
  }

  // ── Stop ──────────────────────────────────────────────────────────────────
  // Only close channels that currently have live PWM.  In normal operation
  // that's at most 1–2 channels — nowhere near enough to spike the rail.
  // This prevents the brownout that was corrupting PCA9685 registers and
  // causing releaseServo() to silently fail.
  else if (action == "stop") {
    uint8_t closed = 0;
    for (uint8_t ch = 0; ch <= 20; ch++) {
      if (servoHasPWM[ch]) {
        closeServo(ch);
        closed++;
      }
    }
    Serial.printf("[STOP]  %d channel(s) were active; settling before release\n", closed);
    stateStartTime = now;
    currentState   = STATE_STOPPING;
  }

  server.send(200, "text/plain", "OK");
}

void handleServo() {
  if (!server.hasArg("ch") || !server.hasArg("action")) {
    server.send(400,"text/plain","Missing args"); return;
  }
  uint8_t ch     = (uint8_t)server.arg("ch").toInt();
  String  action = server.arg("action");
  if (ch > 20) { server.send(400,"text/plain","Bad channel"); return; }
  if      (action == "open")    openServo(ch);
  else if (action == "close")   closeServo(ch);
  else if (action == "mid")     midServo(ch);
  else if (action == "release") releaseServo(ch);
  server.send(200,"text/plain","OK");
}

void handleSettings() {
  if (server.hasArg("startup")) STARTUP_DELAY_MS      = server.arg("startup").toInt();
  if (server.hasArg("open"))    SERVO_OPEN_TIME_MS    = server.arg("open").toInt();
  if (server.hasArg("inter"))   INTER_SAMPLE_DELAY_MS = server.arg("inter").toInt();
  Serial.printf("[WEB]   Timing — startup:%lu  open:%lu  inter:%lu ms\n",
                STARTUP_DELAY_MS, SERVO_OPEN_TIME_MS, INTER_SAMPLE_DELAY_MS);
  server.send(200,"text/plain","OK");
}

void handleNotFound() { server.send(404,"text/plain","Not found"); }

// ── State machine ──────────────────────────────────────────────────────────
void runStateMachine() {
  uint32_t now     = millis();
  uint32_t elapsed = now - stateStartTime;

  switch (currentState) {

    case STATE_CALIBRATION:
    case STATE_STARTUP_PAUSED:
    case STATE_COMPLETE:
    case STATE_MANUAL:
      break;

    case STATE_STARTUP_COUNTDOWN:
      if (elapsed >= STARTUP_DELAY_MS) {
        Serial.println("[FSM]   Countdown done → opening main");
        openServo(MAIN_SERVO_CH);
        stateStartTime = now;
        currentState   = STATE_OPEN_MAIN;
      }
      break;

    case STATE_OPEN_MAIN:
      if (elapsed >= SERVO_SETTLE_MS) {
        releaseServo(MAIN_SERVO_CH);   // remove if main must actively hold against load
        stateStartTime = now;
        currentState   = STATE_INTER_SAMPLE_WAIT;
        Serial.printf("[FSM]   Main open+idle → %lu ms until sample %d/%d\n",
                      INTER_SAMPLE_DELAY_MS, currentServoIdx+1, SAMPLE_SERVO_COUNT);
      }
      break;

    case STATE_INTER_SAMPLE_WAIT:
      if (elapsed >= INTER_SAMPLE_DELAY_MS) {
        Serial.printf("[FSM]   Inter-sample done → closing main for sample %d\n",
                      currentServoIdx+1);
        closeServo(MAIN_SERVO_CH);
        stateStartTime = now;
        currentState   = STATE_CLOSE_MAIN;
      }
      break;

    case STATE_CLOSE_MAIN:
      if (elapsed >= SERVO_SETTLE_MS) {
        releaseServo(MAIN_SERVO_CH);
        Serial.printf("[FSM]   Main closed → opening Ch%d (sample %d/%d)\n",
                      SERVO_CHANNELS[currentServoIdx],
                      currentServoIdx+1, SAMPLE_SERVO_COUNT);
        openServo(SERVO_CHANNELS[currentServoIdx]);
        sampleReleased = false;
        stateStartTime = now;
        currentState   = STATE_SAMPLE_OPEN;
      }
      break;

    case STATE_SAMPLE_OPEN:
      if (!sampleReleased && elapsed >= SERVO_SETTLE_MS) {
        releaseServo(SERVO_CHANNELS[currentServoIdx]);
        sampleReleased = true;
        // Current should now be back at ~0.13–0.2 A until open time expires.
      }
      if (elapsed >= SERVO_OPEN_TIME_MS) {
        Serial.printf("[FSM]   Ch%d open time done → closing\n",
                      SERVO_CHANNELS[currentServoIdx]);
        closeServo(SERVO_CHANNELS[currentServoIdx]);
        stateStartTime = now;
        currentState   = STATE_CLOSE_SAMPLE;
      }
      break;

    case STATE_CLOSE_SAMPLE:
      if (elapsed >= SERVO_SETTLE_MS) {
        releaseServo(SERVO_CHANNELS[currentServoIdx]);
        currentServoIdx++;
        if (currentServoIdx >= SAMPLE_SERVO_COUNT) {
          if (REPEAT_SEQUENCE) {
            currentServoIdx = 0;
            Serial.println("[FSM]   Sequence complete — repeating");
          } else {
            Serial.println("[FSM]   All samples done → COMPLETE");
            currentState = STATE_COMPLETE;
            break;
          }
        }
        Serial.printf("[FSM]   Sample done → re-opening main for sample %d\n",
                      currentServoIdx+1);
        openServo(MAIN_SERVO_CH);
        stateStartTime = now;
        currentState   = STATE_OPEN_MAIN;
      }
      break;

    // After Stop: wait for the (1–2) just-closed servos to settle, then
    // release all channels.  releaseAll() also clears any stale hasPWM flags.
    case STATE_STOPPING:
      if (elapsed >= SERVO_SETTLE_MS) {
        releaseAll();
        Serial.printf("[FSM]   All released — active channels now: %d (should be 0)\n",
                      activePWMCount());
        currentState = STATE_MANUAL;
      }
      break;
  }
}

// ── Setup / Loop ───────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  memset(servoHasPWM, false, sizeof(servoHasPWM));

  Serial.println("=== Servo pulse table ===");
  for (uint8_t i = 0; i <= 20; i++) {
    chOpenPulse[i]   = map(CH_OPEN_DEG[i], 0, 180, 102, 512);
    uint8_t closeDeg = min((int)CH_OPEN_DEG[i] + CLOSE_OFFSET_DEG, 180);
    chClosePulse[i]  = map(closeDeg, 0, 180, 102, 512);
    Serial.printf("  Ch%2d: open=%3d° (%d)  close=%3d° (%d)\n",
                  i, CH_OPEN_DEG[i], chOpenPulse[i], closeDeg, chClosePulse[i]);
    servoPositions[i] = POS_CLOSE;
  }

  Wire.begin(21, 22);

  pwm0.begin(); pwm0.reset(); delay(100);
  pwm0.setOscillatorFrequency(25000000);
  pwm0.setPWMFreq(SERVO_FREQ);

  pwm1.begin(); pwm1.reset(); delay(100);
  pwm1.setOscillatorFrequency(25000000);
  pwm1.setPWMFreq(SERVO_FREQ);
  delay(100);

  // Staggered close to limit startup inrush, then release all PWM.
  // Target idle: ~0.13–0.2 A (ESP32 WiFi AP + two PCA9685 chips, no servo draw).
  Serial.println("=== Startup: staggered close + release ===");
  for (uint8_t ch = 0; ch <= 20; ch++) {
    closeServo(ch);
    delay(50);
  }
  delay(300);
  releaseAll();
  Serial.printf("=== Ready — %d channels active (should be 0) ===\n", activePWMCount());

  stateStartTime = millis();
  currentState   = STATE_STARTUP_COUNTDOWN;

  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("AP: %s  |  http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  server.on("/",         handleRoot);
  server.on("/status",   handleStatus);
  server.on("/control",  handleControl);
  server.on("/servo",    handleServo);
  server.on("/settings", handleSettings);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("Ready");
}

void loop() {
  server.handleClient();
  runStateMachine();
}