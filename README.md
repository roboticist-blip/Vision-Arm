# 🦾 Gesture-Controlled Robotic Arm — ESP32-CAM + MediaPipe

> Real-time hand tracking on PC → inverse kinematics → servo execution on ESP32-CAM.
> Zero cloud. Zero extra microcontrollers. One WiFi link.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Requirements](#2-hardware-requirements)
3. [Repository Structure](#3-repository-structure)
4. [Architecture & Data Flow](#4-architecture--data-flow)
5. [Wiring Guide](#5-wiring-guide)
6. [Software Setup](#6-software-setup)
7. [Running the System](#7-running-the-system)
8. [Gesture Reference](#8-gesture-reference)
9. [Tuning & Calibration](#9-tuning--calibration)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. System Overview

This project implements a **6-DOF robotic arm** controlled entirely by hand gestures captured through an **ESP32-CAM** module. All heavy computation (computer vision, inverse kinematics) runs on the host PC, while the ESP32-CAM acts as both the **eyes** (camera stream) and **hands** (servo executor).

| Axis | Joint | Description |
|------|-------|-------------|
| 0 | Base | 360° horizontal yaw — tracks left/right hand position |
| 1 | Shoulder (J1) | Vertical pitch — IK computed |
| 2 | Elbow (J2) | Vertical pitch — IK computed |
| 3 | Wrist Pitch (J3) | Follows hand tilt angle |
| 4 | Wrist Roll (J4) | Follows hand roll |
| 5 | Gripper | Binary — open / closed on fist gesture |

**Trigger gesture:** Close 3 or more fingers → arm grips object from your hand.

---

## 2. Hardware Requirements

### Electronics

| Component | Qty | Notes |
|-----------|-----|-------|
| ESP32-CAM (AI-Thinker) | 1 | OV2640 camera module |
| MG996R or SG90 Servos | 6 | MG996R recommended for base & shoulder |
| 5V / 3A Power Supply | 1 | Dedicated for servos — do NOT power from ESP32 |
| USB-TTL Adapter (CH340/CP2102) | 1 | For flashing firmware |
| 1000 µF Capacitor | 1 | Across servo power rail for voltage stabilization |
| PCA9685 I²C Servo Driver *(optional)* | 1 | Use if free GPIOs are insufficient |

### Mechanical

- 3D-printed or aluminium arm frame with 6 pivot points
- M3 screws, servo horns, and linkage rods
- Base platform (at least 15 × 15 cm footprint for stability)

### PC Requirements

- Python 3.9+
- Webcam *not required* — ESP32-CAM is the only camera
- WiFi on same LAN as ESP32-CAM

---

## 3. Repository Structure

```
robotic-arm/
├── esp32cam_firmware/
│   └── esp32cam_firmware.ino     # Arduino sketch — stream + servo WebSocket server
│
├── pc_side/
│   ├── pc_main.py                # Entry point — stream consumer + hand tracking loop
│   ├── arm_kinematics.py         # IK solver + gesture detector (importable module)
├── requirements.txt          # Python dependencies
│
└── README.md                     # This file
```

---

## 4. Architecture & Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│                        HOST PC                              │
│    MJPEG frames  ┌──────────────┐   angles dict             │
│   ◄───────────── │  pc_main.py  │ ──────────────────►       │
│                  │              │                           │
│                  │  MediaPipe   │  arm_kinematics.py        │
│                  │  Hands model │  • 2-link planar IK       │
│                  │              │  • Gesture detection      │
│                  └──────────────┘                           │
│                         │                                   │
│              "90,120,60,90,45,1"  (CSV over WS)             │
└─────────────────────────────┼───────────────────────────────┘
                              │ WebSocket :81
                    ┌─────────▼──────────┐
                    │    ESP32-CAM       │
                    │                    │
                    │  HTTP :80  → MJPEG │ ──► PC (above)
                    │  WS   :81  ← CMDs  │
                    │                    │
                    │  GPIO PWM signals  │
                    └─────────┬──────────┘
                              │
              ┌───────────────┼───────────────┐
           Servo 0        Servo 1–4        Servo 5
           (Base)        (J1–J4)         (Gripper)
```

**Communication protocol:** Plain-text CSV over WebSocket.
Format: `"base,j1,j2,j3,j4,grip"` — e.g. `"90,120,60,90,45,1"`
All values 0–180°. Grip: `0` = open, `1` = close.

---

## 5. Wiring Guide

### Servo Power (Critical)

> ⚠️ **Never power servos from the ESP32's 3.3 V or 5 V pins.**
> Use a dedicated 5 V / 3 A supply with a common GND.

```
External 5V PSU (+) ──────────────────────► Servo VCC (all servos)
External 5V PSU (–) ──┬───────────────────► Servo GND (all servos)
                      └───────────────────► ESP32-CAM GND
```

### GPIO → Servo Signal Mapping

| ESP32-CAM GPIO | Joint | Servo |
|----------------|-------|-------|
| GPIO 12 | Base | Servo 0 |
| GPIO 13 | Shoulder J1 | Servo 1 |
| GPIO 14 | Elbow J2 | Servo 2 |
| GPIO 15 | Wrist Pitch J3 | Servo 3 |
| GPIO 16 | Wrist Roll J4 | Servo 4 |
| GPIO 2 | Gripper | Servo 5 |

> **Note on GPIO 2:** Shared with the onboard flash LED. Either leave the LED disconnected or remap the gripper to another free pin.

### Flashing Header

```
ESP32-CAM IO0 → GND         (hold LOW during flash only)
ESP32-CAM U0TX → USB-TTL RX
ESP32-CAM U0RX → USB-TTL TX
ESP32-CAM GND  → USB-TTL GND
ESP32-CAM 5V   → USB-TTL 5V
```

Disconnect IO0–GND after flashing and press RESET.

---

## 6. Software Setup

### 6.1 Arduino / ESP32-CAM Firmware

1. Install [Arduino IDE](https://www.arduino.cc/en/software) and the **ESP32 board package**
   *(Boards Manager → search "esp32" → install Espressif Systems)*

2. Install required Arduino libraries via Library Manager:
   - `WebSockets` by Markus Sattler (Links2004)
   - `ESP32Servo` by Kevin Harrington

3. Open `esp32cam_firmware/esp32cam_firmware.ino`

4. Edit credentials:
   ```cpp
   const char* SSID     = "YOUR_WIFI_SSID";
   const char* PASSWORD = "YOUR_WIFI_PASSWORD";
   ```

5. Board settings:
   | Setting | Value |
   |---------|-------|
   | Board | AI Thinker ESP32-CAM |
   | Partition Scheme | Huge APP (3MB No OTA) |
   | Upload Speed | 115200 |

6. Flash, open Serial Monitor at **115200 baud** — copy the printed IP address.

### 6.2 PC Python Environment

```bash
git clone https://github.com/roboticist-blip/Vision-Arm.git
cd Vision-Arm
pip install -r requirements.txt
```

Dependencies installed:

| Package | Purpose |
|---------|---------|
| `opencv-python` | MJPEG stream decode + display |
| `mediapipe` | Hand landmark detection |
| `websocket-client` | Send servo commands to ESP32 |
| `numpy` | Numerical utilities |

---

## 7. Running the System

### Step 1 — Power on

1. Power servos from an external 5 V supply
2. Power ESP32-CAM via USB
3. Wait ~5 s for WiFi connection (LED blinks, then stays on)

### Step 2 — Start PC controller

```bash
cd pc_side
python pc_main.py --ip <ESP32_IP>
```

Example:
```bash
python pc_main.py --ip 192.168.1.47
```

Optional flag:
```bash
python pc_main.py --ip 192.168.1.47 --fps 25    # default 20
```

### Step 3 — Control

- Hold your hand in front of the ESP32-CAM (30–60 cm away works best)
- The arm mirrors your hand position in real time
- **Close your fist** (3+ fingers curled) → gripper closes and holds
- **Open hand** → gripper releases
- Press **Q** in the OpenCV window to quit

### Live HUD Overlay

The preview window shows:
```
Base:90  J1:112  J2:60
J3:88  J4:90  Grip: OPEN
```
Green text = open, red text = GRAB active.

---

## 8. Gesture Reference

| Hand Pose | Action |
|-----------|--------|
| Open palm, move left/right | Base rotation |
| Raise / lower wrist | Shoulder + elbow IK |
| Tilt hand forward/back | Wrist pitch (J3) |
| Closed fist (3+ fingers curled) | Gripper CLOSE |
| Open hand | Gripper OPEN |

> Gesture sensitivity and IK reach are tunable — see Section 9.

---

## 9. Tuning & Calibration

All tuning parameters live in **`pc_side/arm_kinematics.py`**.

### Arm Dimensions

```python
L1 = 12.0   # Upper arm length in cm  ← measure your build
L2 = 10.0   # Forearm length in cm    ← measure your build
```

### Reach Scaling

```python
# In hand_to_arm():
reach_scale = 25.0   # cm — how far the arm extends for full hand movement
                     # Increase for wider reach, decrease for finer control
```

### Grip Threshold

```python
# In detect_grab():
return curled >= 3   # 3 = three fingers curled triggers grip
                     # Change to 4 for tighter fist requirement
```

### Camera Resolution & FPS

In `esp32cam_firmware.ino`:
```cpp
config.frame_size   = FRAMESIZE_QVGA;   // 320×240 — low latency
config.jpeg_quality = 15;               // 0 = best, 63 = worst
```
Increase to `FRAMESIZE_VGA` (640×480) for better tracking accuracy at the cost of latency.

---

## 10. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Serial shows IP but stream won't open | Firewall blocking port 80 | Allow inbound on port 80/81 on PC firewall |
| Servos jitter or reset | Insufficient power | Use dedicated 5V/3A PSU; add 1000 µF cap across servo rail |
| `Camera init failed` on boot | Wrong board selected | Select AI-Thinker ESP32-CAM; re-flash |
| Hand not detected | Poor lighting or hand too close | Ensure even front lighting; stay 30–60 cm from camera |
| WebSocket keeps reconnecting | ESP32 under load | Lower `--fps` flag or reduce JPEG quality to free MCU cycles |
| Arm overshoots / vibrates | IK values out of servo range | Verify `L1`, `L2` match physical arm; check `_clamp()` limits |
| GPIO 2 / Gripper doesn't respond | Flash LED conflict | Remap gripper pin to GPIO 4 or use PCA9685 |

---

## License

MIT — use freely, attribution appreciated.

---

*Built with ESP32-CAM · MediaPipe Hands · Python · Arduino*
