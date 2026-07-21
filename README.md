# ESP32-Servo-Control — Servo Valve System

Firmware for the **Servo** sequential water sampler: an ESP32 driving 21 Hiwonder
HPS-2018 servo pinch valves (1 main intake + 20 sample valves) to collect 20
timed water samples during a sealed ocean deployment.

## Hardware

| Part | Role |
|------|------|
| ESP32 DevKitC-32UE | Controller |
| 2× Adafruit PCA9685 | Servo PWM drivers, I²C `0x40` / `0x41` |
| 21× Hiwonder HPS-2018 | Pinch valves (Ch20 = main intake, Ch0–19 = samples) |
| Pololu S13V15F5 | 5 V buck-boost (holds regulation as the SLA sags) |
| 2× Power-Sonic PS-6100 | Battery pack, in parallel |

**Pin map:** I²C on `SDA=21 / SCL=22`; ARM LED on `GPIO27` (→ 2.2 kΩ → LED);
battery sense on `GPIO34` (220 kΩ/100 kΩ divider → 0.3125 × V_batt).

## Firmware variants

There are two builds, each good at one job — flash whichever you need into
`src/main.cpp` (only one `main.cpp` builds at a time in PlatformIO):

- **Bench / interactive** (the original): keeps the WiFi AP up and runs a live
  web UI for calibration and manual valve control. Great on the bench, but it
  never sleeps (~100 mA) and cannot survive a deployment.
- **Deployment** (`deepsleep_main.cpp`): deep-sleeps the inter-sample
  waits with radios off, waking only to take one sample per interval. This is
  the build you seal. It still brings up a pre-seal WiFi arm/calibration UI, so
  you don't have to reflash between bench and deployment.

## Build & flash (PlatformIO)

```bash
pio run                 # build
pio run -t upload       # flash (set upload_port in platformio.ini)
pio device monitor      # serial log @ 115200
```

`platformio.ini` targets `board = esp32dev`, which is correct for the
DevKitC-32UE (the `-32UE` is still a classic ESP32). The Adafruit PWM Servo
Driver library is the dependency that matters for the deployment build.

> **Note:** always type PlatformIO/git commands directly rather than pasting
> from a formatted source — autocorrect and diff views inject characters
> (en-dashes for `--`, curly quotes, a leading `+`) that silently break config
> and command lines.

## Deploying

1. Power on cold → the board comes up in **ARM MODE** (WiFi AP
   `LanderController`, password `lander1234`).
2. Open the web UI, calibrate each valve, and set the timing (startup delay,
   sample open time, inter-sample interval). Confirm the battery reading looks
   healthy.
3. Hit **ARM & DEPLOY**. All valves close, the ARM LED blinks ~45 s — **seal the
   enclosure during this window** — then WiFi drops and the mission begins.
4. There is no remote control once armed. Mission state (next-sample index,
   phase, timing) is stored in flash and survives brownouts, so a power glitch
   at depth resumes instead of restarting.

## Key config (top of the deployment firmware)

| Constant | Default | Notes |
|----------|---------|-------|
| `INTER_SAMPLE_DELAY_MS` | 18 h | Also operator-set at arm time |
| `SERVO_OPEN_TIME_MS` | 5 s | Sample valve open duration |
| `VBATT_CUTOFF_V` | 5.60 V | Below this at wake → park, don't actuate |
| `CLOSE_OFFSET_DEG` | 65 | **Mechanical max — do not increase** |
| `SERVO_RAIL_EN_PIN` | undefined | Optional Q2 rail gate; not needed for budget |

## Confirm before you seal

- **Passive hold under load.** Between moves the firmware cuts PWM and lets each
  valve hold by gear friction. The whole power budget depends on the HPS-2018
  gear train *not* back-driving under a pinched tube at 31 bar, held for a full
  interval. Verify on the bench under representative load before trusting it.
- **Sleep current.** The firmware gets the ESP32 and both PCA9685s to microamps,
  but a stock DevKitC still burns ~10 mA in its onboard LDO + USB chip. If your
  measured budget needs less, that's a bare WROOM-32UE or removing those parts —
  not a firmware change.
- Repack the servo gearboxes with petroleum jelly before sealing. Use PTFE- or
  silicone-jacketed wire only (mineral oil attacks PVC).

## Bench validation

Flash the deployment build, open the serial monitor, and arm with a **short
interval (e.g. 30 s)** to watch a full 20-sample run compressed. Confirm: each
wake logs the correct index and channel; a mid-run reset resumes at the right
index; measured sleep current matches budget; and a valve holds position far
longer than one interval under load. Only then repack and seal.
