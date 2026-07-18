# 5-DOF Robotic Arm — FK/IK Control System

> **ESP32 + PCA9685-based 5-DOF robotic arm with forward/inverse kinematics, web console, computer vision pick-and-place, and dual-arm support**

---

## Table of Contents

- [Overview](#overview)
- [Version History](#version-history)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Dependencies](#software-dependencies)
- [Project Structure](#project-structure)
- [Setup & Installation](#setup--installation)
  - [1. Hardware Assembly](#1-hardware-assembly)
  - [2. Measure Link Lengths](#2-measure-link-lengths)
  - [3. Flash ESP32 Firmware](#3-flash-esp32-firmware)
  - [4. Servo Calibration](#4-servo-calibration)
  - [5. Camera Setup (CV Version)](#5-camera-setup-cv-version)
- [Usage](#usage)
  - [Web Console (Final Version)](#web-console-final-version)
  - [HTML Kinematics GUI (V1)](#html-kinematics-gui-v1)
  - [Python Serial Terminal (V1)](#python-serial-terminal-v1)
  - [Vision Pick-and-Place (CV)](#vision-pick-and-place-cv)
- [Technical Details](#technical-details)
  - [Kinematics Model](#kinematics-model)
  - [DH Parameters](#dh-parameters)
  - [Forward Kinematics](#forward-kinematics)
  - [Inverse Kinematics](#inverse-kinematics)
  - [Servo Smoothing](#servo-smoothing)
  - [Serial Protocol](#serial-protocol)
- [Configuration Reference](#configuration-reference)
- [Troubleshooting](#troubleshooting)

---

## Overview

This project implements a complete control system for a **ROT3U 5-DOF robotic arm** using an ESP32 microcontroller and PCA9685 PWM servo driver with DS3115MG servos. The system provides:

- **Forward Kinematics (FK):** Compute end-effector position from joint angles
- **Inverse Kinematics (IK):** Compute joint angles to reach a desired Cartesian position (closed-form geometric solution)
- **Web Console:** Premium browser-based control interface with Web Serial API (no Python needed)
- **Computer Vision:** AprilTag-based pick-and-place system with homography-based coordinate transformation
- **Dual-Arm Support:** Drive two independent arms from a single ESP32 + PCA9685

### Reference Paper

The kinematic model is based on the DH parameters from:
> Ibrahim & Ali, *"Inverse Kinematic Analysis For A 5 DOF Robotic Arm Using Deep Neural Network,"* IJES vol 11, no 18s, 2025.

**Note:** This implementation uses a **closed-form geometric IK solution** (not the DNN from the paper) for real-time control on microcontrollers.

---

## Version History

### V1 — Foundation
- Single-arm ESP32 + PCA9685 firmware (address `0x40`)
- FK/IK kinematics engine on ESP32
- Single-file HTML/JS kinematics GUI (dark theme, offline — copy commands manually)
- Python serial terminal (CustomTkinter) for manual command entry
- Basic serial protocol (J/P/V/G/H/? commands)

### CV (Computer Vision) — Parallel Development
- Camera calibration pipeline (chessboard pattern, 8×5, 30 mm squares)
- AprilTag detection system (`DICT_APRILTAG_36h11`)
- Full pick-and-place vision system:
  - Workspace calibration via known AprilTag positions
  - Homography-based pixel → world coordinate transformation
  - Object, base, and destination tag recognition
- 41 calibration images captured

### Final — Production Version
- **Firmware updates:**
  - Single-arm PCA9685 address changed to `0x48`
  - Shoulder + elbow servo inversion enabled
  - NEW: Dual-arm firmware (one ESP32 drives 12 servos for 2 arms)
- **Completely redesigned web console:**
  - Multi-file professional web app (HTML + JS + CSS)
  - Sidebar navigation with Control / Connection / Configuration sections
  - Dark / light theme with localStorage persistence
  - Web Serial API (direct browser ↔ ESP32 — no Python needed)
  - Dual-arm support with A/B tabs and topology selector
  - Hardware topology configuration (single / shared board / two boards)
  - Live auto-send mode
  - Per-joint inversion toggles
  - Wiring diagram (SVG, dynamic per topology)
  - Toast notifications, mobile responsive design
- Python serial terminal **dropped** — replaced by browser Web Serial

---

## System Architecture

```
┌──────────────────────────────────────────────────┐
│                   Web Console                     │
│  (arm_console.html/js/css)                        │
│  FK/IK Visualization │ Web Serial API │ Topology  │
└───────────┬──────────────────────────────────────┘
            │ USB Serial (115200 baud)
            ▼
┌──────────────────────────────────────────────────┐
│                    ESP32                           │
│  FK/IK Engine │ EMA Smoothing │ Serial Parser     │
└───────────┬──────────────────────────────────────┘
            │ I²C
            ▼
┌──────────────────────────────────────────────────┐
│              PCA9685 PWM Driver                    │
│  CH0-5 (Arm A) │ CH6-11 (Arm B, dual only)       │
└───────────┬──────────────────────────────────────┘
            │ PWM
            ▼
┌──────────────────────────────────────────────────┐
│          DS3115MG Servos (6 per arm)               │
│  Base │ Shoulder │ Elbow │ Wrist │ Roll │ Gripper │
└──────────────────────────────────────────────────┘
```

---

## Hardware Requirements

| Component | Specification |
|---|---|
| **Robotic Arm** | ROT3U 5-DOF |
| **Microcontroller** | ESP32 development board |
| **Servo Driver** | PCA9685 16-channel I²C PWM driver |
| **Servos** | 6× DS3115MG per arm (12× for dual) |
| **Camera** | USB webcam (for CV version only) |
| **AprilTags** | DICT_APRILTAG_36h11 printed tags (CV version) |
| **Calibration Board** | 8×5 chessboard, 30 mm squares (CV version) |
| **Power** | External 5–6V supply for servos via PCA9685 |

### PCA9685 Channel Assignment

| Channel | Joint | Single Arm | Dual Arm A | Dual Arm B |
|---------|-------|------------|------------|------------|
| 0 | Base | ✓ | ✓ | |
| 1 | Shoulder | ✓ | ✓ | |
| 2 | Elbow | ✓ | ✓ | |
| 3 | Wrist | ✓ | ✓ | |
| 4 | Roll | ✓ | ✓ | |
| 5 | Gripper | ✓ | ✓ | |
| 6 | Base | | | ✓ |
| 7 | Shoulder | | | ✓ |
| 8 | Elbow | | | ✓ |
| 9 | Wrist | | | ✓ |
| 10 | Roll | | | ✓ |
| 11 | Gripper | | | ✓ |

### Default Link Lengths

> ⚠️ These are default values from the paper — **replace with your actual measurements!**

| Link | Symbol | Default (mm) | Description |
|------|--------|-------------|-------------|
| Base height | L0 | 75 | Base to shoulder pivot |
| Shoulder vertical offset | A | 14 | Shoulder vertical offset |
| Shoulder horizontal offset | B | 12 | Shoulder horizontal offset |
| Upper arm | L1 | 102 | Shoulder to elbow |
| Forearm | L2 | 130 | Elbow to wrist |
| Wrist to tip | L34 | 173 | Wrist to end-effector |

---

## Software Dependencies

### Arduino / ESP32

- **Arduino IDE** or **PlatformIO**
- **ESP32 Board Package**
- **Libraries:**
  - `Wire.h` (built-in I²C)
  - `Adafruit_PWMServoDriver.h` ([Adafruit PWM Servo Driver Library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library))

### Python (V1 serial terminal only)

```bash
pip install customtkinter pyserial
```

### Python (CV version)

```bash
pip install opencv-python opencv-contrib-python numpy
```

### Web Console (Final version)

- Modern browser with **Web Serial API** support:
  - ✅ Google Chrome 89+
  - ✅ Microsoft Edge 89+
  - ✅ Opera 76+
  - ❌ Firefox (not supported)
  - ❌ Safari (not supported)
- Google Fonts: **Inter** (UI) + **JetBrains Mono** (code/data)

---

## Project Structure

```
Robotic_Arm/
│
├── V1/                                 # Version 1 — Foundation
│   ├── arm_kinematics_control/
│   │   └── arm_kinematics_control.ino  # ESP32 single-arm firmware (PCA9685 @ 0x40)
│   └── kinematics/
│       ├── arm_kinematics_gui.html     # Single-file FK/IK GUI (dark theme)
│       └── gui.py                      # Python serial terminal (CustomTkinter)
│
├── CV(wroked_Till_Now)/                # Computer Vision — Pick & Place
│   ├── calib.py                        # Camera calibration (original)
│   ├── calib_fixed.py                  # Camera calibration (improved error handling)
│   ├── detect.py                       # Live AprilTag detector
│   ├── vision.py                       # Full pick-and-place vision system
│   ├── inspect_images.py              # Image inspection utility
│   ├── camera.npz                      # Saved camera calibration data
│   └── Check_pix/                      # 41 calibration JPEG images
│
├── Final/                              # Final — Production Version
│   ├── arm_kinematics_control/
│   │   └── arm_kinematics_control.ino  # ESP32 single-arm firmware (PCA9685 @ 0x48)
│   ├── arm_kinematics_control_dual/
│   │   └── arm_kinematics_control_dual.ino  # ESP32 dual-arm firmware
│   ├── arm_console.html               # Premium web console (HTML)
│   ├── arm_console.js                  # Console logic (FK/IK, Serial, topology)
│   └── arm_console.css                 # Design system (dark/light themes)
│
└── README.md                           # This file
```

---

## Setup & Installation

### 1. Hardware Assembly

Assemble the ROT3U arm following manufacturer instructions. Connect servos to PCA9685 channels as shown in the channel assignment table above. Wire PCA9685 I²C to ESP32 (SDA → GPIO 21, SCL → GPIO 22).

### 2. Measure Link Lengths

Measure your arm's actual link lengths (L0, A, B, L1, L2, L34) with calipers and update the values in:
- **Firmware:** `LinkConfig` struct in the `.ino` file
- **Web Console:** Configuration panel link length editor

### 3. Flash ESP32 Firmware

**Single Arm:**
```
Final/arm_kinematics_control/arm_kinematics_control.ino
```

**Dual Arm (two arms, one ESP32):**
```
Final/arm_kinematics_control_dual/arm_kinematics_control_dual.ino
```

1. Open in Arduino IDE
2. Install **Adafruit PWM Servo Driver Library**
3. Select ESP32 board, upload
4. Serial Monitor at 115200 → should print joint angles and `ready`

### 4. Servo Calibration

For each servo, determine:
- `pulseAt0` — PCA9685 tick count at 0° (default: 102, ~500 µs)
- `pulseAt180` — PCA9685 tick count at 180° (default: 512, ~2400 µs)
- `angleMin` / `angleMax` — safe mechanical range
- `invert` — set `true` if servo is mounted in reverse

Edit the `SERVO_CAL[]` array in the firmware.

### 5. Camera Setup (CV Version)

1. Print AprilTag markers (DICT_APRILTAG_36h11):
   - IDs 0, 1, 2, 3 → workspace corners at known positions
   - ID 5 → workspace center
   - ID 10 → object to pick
   - ID 15 → arm base
   - IDs 20+ → destination positions
2. Run camera calibration:
   ```bash
   python calib_fixed.py
   ```
3. Place calibration images in `Check_pix/` folder or capture live

---

## Usage

### Web Console (Final Version)

1. Open `Final/arm_console.html` in **Chrome** or **Edge**
2. Click **Connect** → select ESP32 COM port
3. Select **Topology** (single arm / two arms one board / two arms two boards)
4. Choose control mode:
   - **FK Mode:** 5 joint sliders (θ₁–θ₅) for direct angle control
   - **IK Mode:** Enter target Cartesian position (Px, Py, Pz), approach angle (ψ), elbow up/down
5. Enable **Auto-Send** for live servo updates on every slider change
6. Use the **Configuration** section to edit link lengths and inversion flags

### HTML Kinematics GUI (V1)

Open `V1/kinematics/arm_kinematics_gui.html` in any browser. This is an offline tool — computed serial commands must be copied manually and sent via a serial terminal.

### Python Serial Terminal (V1)

```bash
cd V1/kinematics
python gui.py
```

Select COM port, set baud to 115200, connect, and type commands manually.

### Vision Pick-and-Place (CV)

```bash
cd "CV(wroked_Till_Now)"
python detect.py          # Live AprilTag detection test
python vision.py          # Full vision system with workspace mapping
```

---

## Technical Details

### Kinematics Model

The arm has **5 revolute joints** (base rotation, shoulder, elbow, wrist, roll) plus a gripper.

### DH Parameters

Based on Ibrahim & Ali (2025):

| Joint | θ | d | a | α |
|-------|---|---|---|---|
| 1 (Base) | θ₁ | L0 | 0 | -90° |
| 2 (Shoulder) | θ₂ | 0 | L1 | 0° |
| 3 (Elbow) | θ₃ | 0 | L2 | 0° |
| 4 (Wrist) | θ₄ | 0 | L34 | 0° |
| 5 (Roll) | θ₅ | 0 | 0 | 0° |

### Forward Kinematics

Decoupled geometric approach:

$$P_x = \cos\theta_1 \cdot r, \quad P_y = \sin\theta_1 \cdot r, \quad P_z = L_0 + A + h$$

where:
$$r = B + L_1\cos\theta_2 + L_2\cos(\theta_2 + \theta_3) + L_{34}\cos(\theta_2 + \theta_3 + \theta_4)$$
$$h = L_1\sin\theta_2 + L_2\sin(\theta_2 + \theta_3) + L_{34}\sin(\theta_2 + \theta_3 + \theta_4)$$

### Inverse Kinematics

Closed-form geometric solution:

1. **Base rotation:** $\theta_1 = \text{atan2}(P_y, P_x)$
2. **Wrist position:** subtract L34 contribution in approach direction
3. **Elbow angle:** law of cosines on triangle formed by L1, L2, and wrist distance
4. **Shoulder angle:** geometric from wrist position and elbow angle
5. **Wrist angle:** from desired approach angle minus shoulder and elbow
6. **Elbow up/down** selection available

Includes reachability checking and clamping for out-of-range targets.

### Servo Smoothing

- **Exponential Moving Average (EMA)** with α = 0.15
- **Dead zone:** 0.15° (prevents jitter from servo noise)
- **Update interval:** 20 ms

### Serial Protocol

| Command | Format | Description |
|---------|--------|-------------|
| Joint angles | `J θ1 θ2 θ3 θ4 θ5` | Direct joint control |
| Cartesian pose | `P Px Py Pz ψ elbow` | IK to position (elbow: 0=up, 1=down) |
| Vision pose | `V Px Py Pz` | IK with fixed -90° approach angle |
| Gripper | `G angle` | Gripper servo angle |
| Home | `H` | Move to home position |
| Query | `?` | Report current joint angles and position |

**Dual-arm:** Prefix all commands with arm letter:
```
A J 0 30 -40 -10 0       # Arm A joint angles
B P 250 0 150 -30 90     # Arm B Cartesian pose
```

**Home position:** θ₁=0°, θ₂=30°, θ₃=-40°, θ₄=-10°, θ₅=0°, Gripper=90°

---

## Configuration Reference

### Firmware (`arm_kinematics_control.ino`)

| Parameter | V1 Default | Final Default | Description |
|-----------|-----------|---------------|-------------|
| PCA9685 I²C Address | `0x40` | `0x48` (single) / `0x40` (dual) | I²C address |
| Baud Rate | 115200 | 115200 | Serial baud rate |
| EMA Alpha | 0.15 | 0.15 | Smoothing factor |
| Dead Zone | 0.15° | 0.15° | Jitter threshold |
| Update Interval | 20 ms | 20 ms | Servo update period |
| Shoulder Invert | `false` | `true` | Reversed servo mount |
| Elbow Invert | `false` | `true` | Reversed servo mount |

### Web Console (`arm_console.js`)

Topology options:
- **Single Arm:** One arm, one ESP32, one serial port
- **Two Arms, Shared Board:** Two arms on one ESP32+PCA9685 (channels 0-5 and 6-11)
- **Two Arms, Separate Boards:** Two arms on separate ESP32s, two serial ports

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Arm doesn't move | Check external power to PCA9685, verify I²C wiring (SDA/SCL) |
| Wrong direction | Set `invert = true` for that servo in firmware, or use joint inversion toggle in web console |
| IK unreachable | Target is outside arm's workspace — check link lengths match physical arm |
| Web Serial not available | Use Chrome or Edge; ensure `chrome://flags/#enable-experimental-web-platform-features` if needed |
| Servos jitter at limits | Reduce `angleMax` or increase dead zone in firmware |
| Camera not found (CV) | Check camera index, try backends DSHOW → MSMF → ANY |
| AprilTag not detected | Ensure proper lighting, marker size, and dictionary (`DICT_APRILTAG_36h11`) |

---

*Project developed during internship — 5-DOF Robotic Arm FK/IK Control System with Web Console and Computer Vision*
