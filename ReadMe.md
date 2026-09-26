# PD Line Follower — ESP32-S3 · XLine 16 · N20

A competition line-following robot built on an **ESP32-S3**, an **XLine 16** analog IR sensor
array and two **N20 gear motors 5volts 1000 RPM** with tb6612 motor driver. It runs a PD controller with a sensor-driven state machine
for forks, acute tips, 90° corners, zigzags, inverted (white-on-black) zones, line gaps and a
full-black finish pad.

Every tuning parameter can be changed **live over Wi-Fi** from a phone. No reflashing is needed
between runs.

---

## Table of Contents
- [Features](#features)
- [Hardware](#hardware)
- [Wiring](#wiring)
- [Sensor Layout](#sensor-layout)
- [Software Setup](#software-setup)
- [Web Tuner](#web-tuner)
- [First-Run Calibration](#first-run-calibration)
- [How It Works](#how-it-works)
- [Telemetry States](#telemetry-states)
- [Parameter Reference](#parameter-reference)
- [Tuning Guide](#tuning-guide)
- [Troubleshooting](#troubleshooting)
- [Revision History](#revision-history)
- [License](#license)

---

## Features

- **PD steering** on a squared-weight centroid, measured in channel units (error −7 … +7)
- **Straight-line boost**: extra speed when centered, normal speed while correcting
- **Error-hold**: keeps steering with the last correction when the line slides off the side
- **Edge-latch routing**: the outer sensors detect a branch *before* the line is lost, then the
  robot pivots toward the latched side (acute tips, diodes, 90° corners)
- **Sticky Y-bias**: wide clusters commit to one edge until the line narrows again
- **Fork selection** with position continuity (ignores far-away line jumps)
- **Inverted-zone detection**: automatic polarity flip on entry and exit
- **Zone-gap window**: after leaving an inverted zone, a missing line triggers a forward probe instead of reversing
- **Lost-line recovery**: forward gap probe, then reverse until the center sensor is on the line, then spin at the vertex
- **Zigzag mode**: detects rapid alternating corners and drops the base speed
- **Kick-start** on motor direction changes to overcome N20 static friction
- **Full-black finish pad** detection with automatic stop
- **Live web UI** with a sensor bar, state, polarity, edge latches, PWM output and loop timing

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | ESP32-S3 dev board |
| Line sensor | XLine 16 IR array: 16 channels behind a 16:1 analog multiplexer (S0–S3 select, one analog output) |
| Motors | 2 × N20 micro gear motors (`5 volts 1000 RPM`) |
| Motor driver | tb6612 drive motor driver |
| Power | 12 Volts Lipo 1300mAH with a buck converter for a 5volts output |
| Start button | Momentary push button to GND (uses the internal pull-up) |

---

## Wiring

### Motor driver

| Driver pin | ESP32-S3 GPIO | Function |
|------------|---------------|----------|
| IN1 | 6 | Left motor direction A |
| IN2 | 7 | Left motor direction B |
| ENA | 9 | Left motor PWM (1 kHz, 8-bit) |
| IN3 | 4 | Right motor direction A |
| IN4 | 5 | Right motor direction B |
| ENB | 8 | Right motor PWM (1 kHz, 8-bit) |

> If a motor spins backwards, swap its two motor wires on the driver output. There is no need to change the code.

### XLine 16 sensor

| XLine 16 pin | ESP32-S3 GPIO | Function |
|--------------|---------------|----------|
| S0 | 10 | Mux select bit 0 |
| S1 | 11 | Mux select bit 1 |
| S2 | 12 | Mux select bit 2 |
| S3 | 13 | Mux select bit 3 |
| SIG / OUT | 2 | Analog output (ADC, 11 dB attenuation, 0–4095) |
| VCC / GND | 3V3 / GND | Power |

### Other

| Part | GPIO |
|------|------|
| Start/stop button | 1 (to GND, `INPUT_PULLUP`) |

---

## Sensor Layout

```
        LEFT                                        RIGHT
  [14][13][12][11][10][ 9][ 8][ 7][ 6][ 5][ 4][ 3][ 2][ 1][ 0]
   ^^^^^^                                              ^^^^^^
   left edge zone                                right edge zone
                         center = 7.0

  CH15 = center / pivot-axis sensor (read separately)
```

- Channels **0–14** form the tracking bar. **Index 0 is the rightmost** and 14 is the leftmost.
- `error > 0` means the line is to the **left**.
- The outer `EDGE_ZONE` channels on each side act as edge-latch detectors. **Keep all 15 active.**
- **CH15** sits at the robot's pivot axis. Lost-line recovery reverses until CH15 is on the
  line, so the recovery spin happens exactly at the vertex of an acute tip.

---

## Software Setup

1. Install the **Arduino IDE** (2.x recommended).
2. Add the **ESP32 board package, version 3.x**. The code uses the 3.x `ledcAttach()` API and will not compile on 2.x.
3. Select your ESP32-S3 board and port.
4. Open `line_follower/line_follower.ino` and upload.

No external libraries are needed. `WiFi.h` and `WebServer.h` ship with the ESP32 core.

---

## Web Tuner

1. Power on the robot.
2. Connect your phone or laptop to Wi-Fi **`LineFollower`** (password `12345678`).
3. Open **http://192.168.4.1**.

The page refreshes every 300 ms and shows:

| Field | Meaning |
|-------|---------|
| Sensor bar | `B` = on the line, `W` = background (polarity-aware) |
| State | Current control state (see [Telemetry States](#telemetry-states)) |
| Status | ON LINE / LOST / FINISHED |
| Polarity | normal / INVERTED |
| Line count / clusters | Active channels / significant clusters |
| Max cluster width | Widest cluster in channels |
| Edge latch L/R | Currently latched branch sides |
| Route side / event | Side the router will pick, and whether a fork event is locked |
| Track pos | Current tracked line position (0–14) |
| Center CH15 | Pivot-axis sensor state |
| Error / Corr | PD error and correction |
| L / R PWM | Motor commands (−255 … 255) |
| Loop µs (max) | Control loop timing |

Every parameter has an input box. Changes apply **immediately** but are **not saved to flash**.
They reset to the code defaults on reboot, so copy any good values back into the code.

Press **START** on the page or the physical button to run or stop. Each start resets all runtime state.

---

## First-Run Calibration

Do these on the actual mat before the first timed run:

1. **`THRESHOLD` / `NOISE`**: place the bar over white, then over the black line, and read
   the raw values. Set `THRESHOLD` between them and `NOISE` just above the white reading.
2. **`INV_WHITE`**: park the bar **inside an inverted zone** and read the raw values of the white line
   on black. Set `INV_WHITE` between the white-line and black-background readings.
3. **Direction check**: run slowly (`base` ≈ 80) and confirm that a line on the left makes the robot
   turn left. If it turns the wrong way, swap the motor wires. Do not flip signs in the code.
4. **`Y_SIDE`**: set the default fork preference (1 = left, 2 = right, 0 = nearest).

---

## How It Works

Each loop (about 4 ms plus sensor read time), `control()` runs through these stages in priority
order. The first stage that takes control returns.

| # | Stage | What it does |
|---|-------|--------------|
| 0 | Edge scan | Every cycle: latches a side if an outer edge sensor sees line while the main line is centered |
| 1 | Black-back | Reverses off the finish pad (only if `FULL_BLACK_STOP = 0`) |
| 2 | Inverted probe | After an all-line event, drives straight to decide between a zone entry and the finish pad |
| 3 | Pivot | Continues an in-place corner pivot until the line is near center |
| 4 | Edge turn | Continues a latched-side pivot until the line overshoots center, then hands back to PD |
| 5 | All-line | Crossing blip (`XING`), or confirmed zone entry/exit (`INV?` / `INV-OUT`) |
| 6 | Edge-turn trigger | Line lost with a fresh latch: start an edge turn |
| 7 | Line lost | Zone-gap probe → error-hold → gap probe → reverse to vertex → spin search |
| 8 | Target selection | Chooses a cluster by continuity, fork side, or sticky Y-bias edge |
| 9 | Pivot entry | A large error starts a corner pivot and counts toward zigzag mode |
| 10 | PD | `corr = Kp·e + Kd·(e − e_last)`, with boost and a reverse limit |

### Routing priority (which way to turn)

1. **Zone-gap window active**: no turn, drive forward
2. **Freshest edge latch**: the side that was just seen branching
3. **Event-held side**: the side locked when the fork event opened
4. **`Y_SIDE`** fallback, or the last-seen line position during spin recovery

There is no hardcoded turn sequence. Every routing decision comes from the sensors.

---

## Telemetry States

| State | Meaning |
|-------|---------|
| `stopped` | Not running |
| `PD` / `PD-ZZ` | Normal tracking / tracking in zigzag mode |
| `FORK` | Two candidate lines, picking by route side |
| `Y-BIAS` | Hugging one edge of a wide cluster |
| `SNAP-TURN` | Both lines visible with center on: boosted gains |
| `CORNER_L/R`, `ZZ_L/R` | In-place pivot (normal / zigzag) |
| `EDGE_L/R` | Edge-latch turn in progress |
| `XING` | Crossing or checkpoint bar being ridden through |
| `INV?` | All-line confirmed, probing: zone or finish? |
| `INV-IN` | Entered an inverted zone |
| `INV-OUT` | Left an inverted zone |
| `ZONE-GAP` | Expected gap after a zone exit, driving forward |
| `ERR-HOLD` | Line slid off the side, holding the last correction |
| `GAP-FWD` | Centered line vanished, probing forward for a gap |
| `LOST-BACK` | Reversing until CH15 finds the line |
| `LOST-SPIN` / `LOST-FWD` | Spinning / creeping forward to reacquire |
| `BLACK-BACK` | Reversing off the finish pad |
| `FINISH` | Finish pad confirmed, motors stopped |

---

## Parameter Reference

All parameters are live-tunable from the web UI. Distances and positions are in **channel units** (0–14).

### PD and speed
| Param | Default | Description |
|-------|---------|-------------|
| `Kp` | 40.0 | Proportional gain |
| `Kd` | 165.0 | Derivative gain |
| `base` | 150 | Base PWM |
| `BOOST_XP` | 30 | Extra PWM when centered (0 = off) |
| `BOOST_BAND` | 0.5 | Error range that counts as centered |
| `SPEED_LIMIT` | 255 | Maximum forward PWM |
| `MAX_REVERSE` | 255 | Maximum reverse PWM (scaled with the effective base) |
| `ERR_HOLD_MS` | 250 | How long to hold the last correction after a side loss |

### Sensors
| Param | Default | Description |
|-------|---------|-------------|
| `THRESHOLD` | 1800 | Raw value above which a channel is on the black line |
| `NOISE` | 1000 | Raw floor subtracted before weighting |
| `MUX_SETTLE_US` | 60 | Settling delay after switching mux channels |

### Pivot and zigzag
| Param | Default | Description |
|-------|---------|-------------|
| `PIVOT_THRESHOLD` | 5.0 | Error that triggers an in-place pivot |
| `PIVOT_SPEED` | 90 | Pivot PWM |
| `PIVOT_TIMEOUT_MS` | 900 | Pivot give-up time |
| `ZZ_BASE` | 110 | Base PWM in zigzag mode |
| `ZZ_TRIGGER` | 2 | Corners within the window that enable zigzag mode |
| `ZZ_WINDOW_MS` | 150 | Corner-streak window |
| `ZZ_EXIT_MS` | 75 | Leave zigzag mode after this long without a corner |
| `ZZ_CENTER_MS` | 75 | …or after this long centered |

### Inverted zones and crossings
| Param | Default | Description |
|-------|---------|-------------|
| `INV_WHITE` | 1800 | Raw value below which a channel is on the white line (inverted). **Calibrate on the mat** |
| `XING_MIN` | 8 | Active channels that count as all-line |
| `INV_CONFIRM_MS` | 280 | All-line dwell time before treating it as a zone or pad (not a crossing) |
| `XING_SPEED` | 75 | PWM while crossing or probing |
| `XING_GAIN` | 1.0 | Steering gain multiplier while crossing |
| `INV_PROBE_MS` | 800 | Probe time to decide between zone and finish |

### Routing and clusters
| Param | Default | Description |
|-------|---------|-------------|
| `Y_SIDE` | 1 | Fallback fork side: 1 = left, 2 = right, 0 = nearest |
| `Y_MIN` | 4 | Cluster width that arms the sticky Y-bias |
| `Y_EXIT_W` | 3 | Width at or below which Y-bias starts releasing |
| `Y_EXIT_MS` | 125 | Narrow time needed to release Y-bias |
| `SIGN_GAIN` | 0.6 | PD gain multiplier in fork and Y-bias modes |
| `SIGN_CONFIRM_MS` | 50 | Wide time needed to latch Y-bias |
| `SIGN_MIN_FRAC` | 0.12 | Minimum weight fraction for a cluster to count |
| `JUMP_MAX` | 6.0 | Maximum position jump accepted as the same line |
| `TURN_COOLDOWN_MS` | 125 | How long a fork event keeps its side locked |
| `DIR_SWAP` | 0 | 1 = mirror all routed directions |

### Edge latch
| Param | Default | Description |
|-------|---------|-------------|
| `EDGE_ZONE` | 2 | Outer channels per side used as edge detectors (1–4) |
| `EDGE_CONFIRM_MS` | 100 | Debounce time before latching |
| `EDGE_HOLD_MS` | 200 | How long a latch stays valid |
| `EDGE_TURN_SPEED` | 80 | Edge-turn pivot PWM |
| `EDGE_TURN_MS` | 200 | Edge-turn timeout, after which the vertex spin takes over |
| `EDGE_BLANK_MS` | 60 | Blind time at the start of a turn |
| `EDGE_EXIT_POS` | 0.25 | Overshoot past center required to exit the turn |
| `EDGE_SWAP` | 0 | 1 = mirror the edge-turn direction only |

### Lost-line recovery
| Param | Default | Description |
|-------|---------|-------------|
| `GAP_FWD_MS` | 250 | Forward probe time for a centered gap |
| `GAP_SPEED` | 80 | PWM for probing and reversing |
| `GAP_EXPECT_MS` | 800 | Zone-gap window after an inverted-zone exit |
| `LOST_BACK_MS` | 900 | Maximum reverse time looking for CH15 |
| `SPIN_BLANK_MS` | 225 | Blind time at the start of a spin |
| `SPIN_SPEED` | 90 | Spin PWM |
| `SPIN_360_MS` | 2500 | Spin duration before a forward creep |
| `SEARCH_FWD_MS` | 300 | Forward creep between spins |

### Motor kick and finish
| Param | Default | Description |
|-------|---------|-------------|
| `KICK_PWM` | 220 | PWM burst on a direction change (helps N20 static friction) |
| `KICK_MS` | 60 | Burst duration |
| `KICK_MIN_CMD` | 60 | Minimum command that triggers a kick |
| `FULL_BLACK_STOP` | 1 | 1 = stop on the finish pad, 0 = back off it |
| `FULL_BLACK_BACK_MS` | 400 | Back-off time when not stopping |

---

## Tuning Guide

1. **Start slow**: `base` ≈ 100, `BOOST_XP` = 0. Tune `Kp` until the robot follows curves without
   oscillating, then raise `Kd` to damp the wobble.
2. **Raise speed** in steps of about 10, re-tuning `Kd` each time. Enable `BOOST_XP` once straights are stable.
3. **Corners**: if the robot overshoots 90° turns, lower `PIVOT_THRESHOLD` or `PIVOT_SPEED`.
4. **Forks and tips**: watch *Edge latch L/R* on the UI while pushing the robot through by hand.
   Latches should appear just before the branch. Adjust `EDGE_CONFIRM_MS` or `EDGE_ZONE`.



**Author:** Yazan Abubakir 
