"""
arm_kinematics.py
-----------------
- 2-link planar IK for J1/J2 (shoulder + elbow)
- Hand tracking → arm angles mapping
- Gesture detection (grab trigger)

Arm config (tune to your physical build):
  L1 = upper arm length  (cm)
  L2 = forearm length    (cm)
  The base rotates in the horizontal plane.
  J1 (shoulder) and J2 (elbow) operate in the vertical plane.
  J3/J4 are wrist pitch/roll, mapped linearly from hand orientation.
"""

import math
import numpy as np

# ── Physical constants ────────────────────────────────────────────────────────
L1 = 12.0   # cm  upper arm
L2 = 10.0   # cm  forearm

SERVO_MIN = 0
SERVO_MAX = 180

# ── IK ────────────────────────────────────────────────────────────────────────
def ik_2link(x: float, y: float) -> tuple[int, int]:
    """
    2-link planar IK.
    x = horizontal reach (cm), y = vertical height (cm).
    Returns (j1_deg, j2_deg) or None if out of reach.
    """
    d = math.hypot(x, y)
    d = min(d, L1 + L2 - 0.5)   # clamp to reachable

    cos_j2 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_j2 = max(-1.0, min(1.0, cos_j2))
    j2 = math.acos(cos_j2)      # elbow angle (0 = straight)

    alpha = math.atan2(y, x)
    beta  = math.atan2(L2 * math.sin(j2), L1 + L2 * math.cos(j2))
    j1    = alpha - beta

    j1_deg = int(math.degrees(j1))
    j2_deg = int(math.degrees(j2))

    j1_deg = _clamp(j1_deg + 90, SERVO_MIN, SERVO_MAX)   # offset: 90° = horizontal
    j2_deg = _clamp(j2_deg,      SERVO_MIN, SERVO_MAX)
    return j1_deg, j2_deg

def _clamp(v, lo, hi):
    return max(lo, min(hi, int(v)))

# ── Hand landmark → arm angles ─────────────────────────────────────────────────
def hand_to_arm(landmarks, frame_w: int, frame_h: int,
                reach_scale: float = 25.0) -> dict:
    """
    landmarks: list of 21 MediaPipe hand landmark objects.
    Returns dict: base, j1, j2, j3, j4, grip  (all int degrees / 0|1)
    """
    # Wrist = landmark 0, Middle MCP = landmark 9
    wrist  = landmarks[0]
    mid    = landmarks[9]

    # Normalised coords (0-1) → pixel → world approximation
    wx = wrist.x * frame_w
    wy = wrist.y * frame_h
    mx = mid.x   * frame_w
    my = mid.y   * frame_h

    # ── Base: horizontal position of wrist mapped to 0-180°
    base = _clamp(int(wrist.x * 180), 0, 180)

    # ── Reach / height from wrist position
    # Map normalised y (0=top, 1=bottom) → vertical height in cm
    norm_x = wrist.x - 0.5          # -0.5 .. +0.5
    norm_y = 0.5 - wrist.y          # flip: +ve = up

    reach  = norm_x * reach_scale   # cm
    height = norm_y * reach_scale   # cm

    j1, j2 = ik_2link(abs(reach) + 5, height)   # min reach offset

    # ── Wrist orientation from wrist→mid vector
    dx = mx - wx
    dy = my - wy
    hand_angle = math.degrees(math.atan2(-dy, dx))  # flip y (image coords)

    j3 = _clamp(int(hand_angle + 90), 0, 180)       # pitch
    j4 = 90                                           # roll: keep neutral

    # ── Grip gesture
    grip = detect_grab(landmarks)

    return dict(base=base, j1=j1, j2=j2, j3=j3, j4=j4, grip=int(grip))


# ── Gesture: closed fist = grab ───────────────────────────────────────────────
# Finger tip landmarks: 4(thumb),8(index),12(middle),16(ring),20(pinky)
# Finger MCP landmarks: 2,       5,       9,         13,      17
TIPS = [8, 12, 16, 20]
MCPS = [5,  9, 13, 17]

def detect_grab(landmarks) -> bool:
    """True if 3+ fingers are curled (tip below MCP in image y)."""
    curled = 0
    for tip_i, mcp_i in zip(TIPS, MCPS):
        if landmarks[tip_i].y > landmarks[mcp_i].y:   # tip below knuckle
            curled += 1
    return curled >= 3


# ── Format command string for WebSocket ───────────────────────────────────────
def angles_to_cmd(a: dict) -> str:
    return f"{a['base']},{a['j1']},{a['j2']},{a['j3']},{a['j4']},{a['grip']}"
