# Cyberarm — Arm Mirroring System

Real-time human arm mirroring to a 6-DOF Piper robot arm using an Orbbec depth camera and MediaPipe Pose.

A person stands in front of the camera, and the Piper robot replicates their arm movements in real time.

## System Architecture

```text
┌──────────────────────────────────────────────────────────────────────┐
│                         arm_mirror.py                                  │
├──────────────────────────────────────────────────────────────────────┤
│                                                                        │
│  Orbbec Camera (color stream @ 30fps)                                  │
│       │                                                                │
│       ▼                                                                │
│  MediaPipe Pose → pose_world_landmarks (33 body points, 3D meters)     │
│       │                                                                │
│       ▼                                                                │
│  angle_utils.py: landmarks_to_piper_joints()                           │
│    • J1: shoulder horizontal rotation (atan2 in XZ plane)              │
│    • J2: shoulder pitch (torso-vertical vs upper-arm angle)            │
│    • J3: elbow bend (upper-arm vs forearm angle, negative range)       │
│    • J4–J6: wrist DOF (phase 2)                                       │
│       │                                                                │
│       ▼                                                                │
│  Smoothing (EMA) → Velocity Limiting → Joint Clamping                  │
│       │                                                                │
│       ▼                                                                │
│  piper_sdk: MotionCtrl_2() + JointCtrl(j1..j6) → CAN bus → Robot      │
│                                                                        │
└──────────────────────────────────────────────────────────────────────┘
```

## Repository Structure

```text
SRC/
├── README.md                    # This file
├── requirements.txt             # Python dependencies (mediapipe, opencv, numpy)
├── sdks/
│   ├── pyorbbecsdk/             # Orbbec camera SDK (compiled C++ wrapper)
│   └── piper_sdk/               # Piper robotic arm SDK
└── src/
    ├── arm_mirror.py            # Main application — camera + pose + arm control
    ├── angle_utils.py           # Math: landmark→angle conversion, smoothing, safety
    ├── hello_orbbec.py          # Camera test & live viewer
    ├── hello_piper.py           # Arm control test script
    ├── teleop.py                # Master/slave teleoperation
    ├── hello_can.py             # Raw CAN bus test
    └── setup_macos_can.py       # CAN adapter detection (macOS)
```

## File Documentation

### `src/arm_mirror.py` — Main Application

The entry point that ties everything together. Initializes the camera, MediaPipe, and robot arm, then runs a continuous loop:

1. Captures a color frame from the Orbbec camera
2. Feeds it to MediaPipe Pose to get 3D body landmarks
3. Extracts shoulder/elbow/wrist positions and calculates joint angles
4. Applies smoothing and safety limits
5. Sends the angles to the Piper arm over CAN bus
6. Displays the camera feed with landmark overlay and angle readouts

**CLI Options:**
| Flag | Default | Description |
|------|---------|-------------|
| `--no-arm` | off | Vision-only debug mode (no robot connection) |
| `--can` | `can0` | CAN bus channel |
| `--speed` | `50` | Arm speed as % of max (lower = safer) |
| `--alpha` | `0.3` | Smoothing factor (0=no change, 1=no smoothing) |
| `--max-step` | `5.0` | Max degrees of change per frame (velocity limit) |
| `--arm` | `right` | Which human arm to track (`left` or `right`) |

### `src/angle_utils.py` — Math & Safety Layer

Pure functions with no hardware dependencies. Handles:

- **`landmarks_to_piper_joints(world_landmarks, side)`** — Converts MediaPipe's 3D landmark coordinates into 6 Piper joint angles (degrees) using vector geometry
- **`AngleSmoother`** — Exponential moving average filter to reduce jitter from noisy pose estimates
- **`VelocityLimiter`** — Caps per-frame angle changes to prevent violent arm jerks
- **`clamp_to_limits(angles)`** — Enforces joint range limits (with 70% safety margin)
- **`check_visibility(landmarks, side)`** — Gates tracking on landmark confidence scores

### Joint Mapping

| Piper Joint | Range | Human Motion | How It's Calculated |
|-------------|-------|-------------|---------------------|
| J1 (base rotation) | ±150° | Shoulder abduction | atan2 of shoulder→elbow projected on horizontal plane |
| J2 (shoulder pitch) | 0–180° | Arm raise/lower | Angle between torso vertical and upper arm |
| J3 (elbow) | -170–0° | Elbow bend | Angle at elbow (0°=straight, -170°=fully bent) |
| J4 (forearm roll) | ±100° | Forearm rotation | Phase 2 |
| J5 (wrist pitch) | ±70° | Wrist flex/extend | Phase 2 |
| J6 (wrist yaw) | ±120° | Wrist deviation | Phase 2 |

### Safety Mechanisms

- Joint angles clamped to 70% of full mechanical range
- Velocity limiter prevents > 5°/frame movement (~150°/s at 30fps)
- Tracking requires landmark visibility > 60% for all required joints
- If pose is lost for > 1 second, arm returns to home position
- Arm speed capped via `MotionCtrl_2` speed parameter
- Clean shutdown on Ctrl+C: return to zero → disable arm

---

## Setup Guide

This section covers how to initialize the repository layout, download and compile the hardware SDKs, and integrate them with the application code.

### Prerequisites

```bash
sudo apt update
sudo apt install build-essential cmake git python3-dev python3-pip can-utils
```

### Python Dependencies

```bash
pip3 install -r requirements.txt
```

This installs `mediapipe`, `opencv-python`, and `numpy`. The Piper SDK and pyorbbecsdk are installed separately (see below).

### CAN Bus Setup (run once per boot)

```bash
sudo ip link set can0 up type can bitrate 1000000
```

### Orbbec SDK (pyorbbecsdk)

Clone and compile the Orbbec Python SDK:

```bash
git clone https://github.com/orbbec/pyorbbecsdk.git
cd pyorbbecsdk
git checkout v2-main
pip3 install "pybind11>=2.10"
rm -rf build && mkdir build && cd build
cmake -Dpybind11_DIR=$(pybind11-config --cmakedir) -DBUILD_EXAMPLES=OFF ..
make -j$(nproc)
make install
cd ../..
```

### Piper SDK

Install the Piper robot arm SDK:

```bash
pip3 install piper_sdk
```

Or clone into `sdks/piper_sdk/` and install from source.

---

## Quick Start

```bash
# 1. Activate CAN bus
sudo ip link set can0 up type can bitrate 1000000

# 2. Test camera only (no robot needed)
python3 src/arm_mirror.py --no-arm

# 3. Full arm mirroring (robot connected)
python3 src/arm_mirror.py --speed 20

# 4. Increase speed once comfortable
python3 src/arm_mirror.py --speed 50
```

---

## Component Verification

Test each piece in isolation before running the full system. Each step below is independent — you don't need the other components to pass.

### 1. Camera (Orbbec)

Verifies the camera is connected, streaming, and producing valid color + depth data.

```bash
python3 src/hello_orbbec.py
```

**Expected output:**
- Terminal prints device name, serial number, firmware, and stream resolutions
- Two windows open: "Orbbec - Color" (live RGB feed) and "Orbbec - Depth" (colorized depth map with center distance)
- Press Q or ESC to quit

**If it fails:** Check USB connection, try a different port, ensure pyorbbecsdk is built.

### 2. CAN Bus

Verifies the CAN adapter is active and can send/receive messages.

```bash
# Activate CAN (requires sudo)
sudo ip link set can0 up type can bitrate 1000000

# Send a test message and listen
python3 src/hello_can.py
```

**Expected output:** Sends a CAN frame and listens for responses. If the Piper is powered on, you'll see heartbeat messages.

**If it fails:** Run `ip link show can0` to confirm the interface is up. Check `dmesg` for USB adapter errors.

### 3. Piper Arm (Motor Control)

Verifies the arm receives commands and moves. **Ensure the arm has clearance before running.**

```bash
python3 src/hello_piper.py
```

**Expected output:**
- Prints firmware version
- Arm enables, moves up and down 5 times
- Prints real-time joint angles (target vs actual)
- Returns to zero and disables

**If it fails:** Check CAN bus is active (step 2), arm is powered on, and no E-stop is engaged.

### 4. Angle Math (No Hardware)

Verifies the landmark-to-joint-angle calculations produce correct values.

```bash
python3 -c "
import sys; sys.path.insert(0, 'src')
from angle_utils import landmarks_to_piper_joints, AngleSmoother, VelocityLimiter
import types

def lm(x, y, z): return types.SimpleNamespace(x=x, y=y, z=z, visibility=1.0)

landmarks = [None] * 33
landmarks[24] = lm(0.0, 0.5, 0.0)   # right hip
landmarks[12] = lm(0.0, 0.0, 0.0)   # right shoulder (above hip)
landmarks[14] = lm(0.3, 0.0, 0.0)   # right elbow (out to side)
landmarks[16] = lm(0.6, 0.0, 0.0)   # right wrist (arm straight)

angles = landmarks_to_piper_joints(landmarks, side='right')
print(f'J1 (base):     {angles[0]:+.1f} deg')
print(f'J2 (shoulder): {angles[1]:+.1f} deg')
print(f'J3 (elbow):    {angles[2]:+.1f} deg')
print()
print('Expected: J2 ~ 90 (arm perpendicular to torso), J3 ~ 0 (straight elbow)')
"
```

**Expected output:** J2 ≈ 90° and J3 ≈ 0° for a straight arm held out to the side.

### 5. Smoothing & Safety (No Hardware)

Verifies that sudden jumps are damped and clamped correctly.

```bash
python3 -c "
import sys; sys.path.insert(0, 'src')
from angle_utils import AngleSmoother, VelocityLimiter, clamp_to_limits

smoother = AngleSmoother(alpha=0.3)
limiter = VelocityLimiter(max_step_deg=5.0)

# Simulate: steady state then sudden 50° jump (like a MediaPipe glitch)
for raw in [[0,0,0,0,0,0], [0,0,0,0,0,0], [50,90,-80,0,0,0]]:
    s = smoother.smooth(raw)
    l = limiter.limit(s)
    c = clamp_to_limits(l)
    print(f'Raw:{raw}  Smoothed:{[round(x,1) for x in s]}  Limited:{[round(x,1) for x in l]}  Clamped:{[round(x,1) for x in c]}')
"
```

**Expected output:** The 50° jump is reduced by the smoother (~15°) and further capped by the velocity limiter (max 5° step). Final values stay within joint limits.

### 6. MediaPipe Pose (Camera, No Arm)

Verifies that pose detection works and angles are calculated from the live camera feed.

```bash
python3 src/arm_mirror.py --no-arm
```

**Expected output:**
- Camera window opens with skeleton overlay drawn on your body
- Joint angles (J1–J6) displayed in the top-left corner, updating in real time
- Status shows "TRACKING" (green) when your arm is visible, "LOST" (red) when not
- Press Q or ESC to quit

**If it fails:** Ensure `mediapipe` is installed (`pip3 install mediapipe`). Stand far enough from the camera for your upper body to be visible.

### 7. Full System (All Components)

Once steps 1–6 all pass, run the complete pipeline at low speed:

```bash
python3 src/arm_mirror.py --speed 20
```

**Expected output:** Robot mirrors your arm movements at 20% speed. Increase `--speed` once you're confident the mapping is correct.

---

## Data Flow Detail

```
Camera Frame (BGR, 640x480 @ 30fps)
    → cv2.cvtColor(BGR→RGB)
    → mediapipe.Pose.process(rgb)
    → pose_world_landmarks.landmark[0..32]  (3D coords in meters)
    → landmarks_to_piper_joints()           (vector math → 6 angles in degrees)
    → AngleSmoother.smooth()                (EMA filter, reduces jitter)
    → VelocityLimiter.limit()               (max 5°/frame change)
    → clamp_to_limits()                     (enforce 70% of joint ranges)
    → degrees_to_millidegrees()             (× 1000, cast to int)
    → piper.JointCtrl(j1, j2, j3, j4, j5, j6)  (CAN bus → motor controllers)
```