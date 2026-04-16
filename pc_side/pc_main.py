"""
pc_main.py
----------
Pipeline:
  1. Pull MJPEG stream from ESP32-CAM
  2. Run MediaPipe Hands on each frame
  3. Compute arm angles via IK (arm_kinematics.py)
  4. Send servo command over WebSocket to ESP32

Usage:
  pip install opencv-python mediapipe websocket-client numpy
  python pc_main.py --ip 192.168.x.x
"""

import cv2
import argparse
import threading
import time
import numpy as np
import websocket

# MediaPipe NEW API
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from arm_kinematics import hand_to_arm, angles_to_cmd

parser = argparse.ArgumentParser()
parser.add_argument("--ip", required=True, help="ESP32-CAM IP address")
parser.add_argument("--fps", type=int, default=20)
args = parser.parse_args()

STREAM_URL = f"http://{args.ip}/stream"
WS_URL     = f"ws://{args.ip}:81"

latest_frame = None
frame_lock   = threading.Lock()
ws_conn      = None
last_cmd     = ""
running      = True

def ws_thread():
    global ws_conn
    while running:
        try:
            ws_conn = websocket.create_connection(WS_URL, timeout=5)
            print(f"[WS] Connected to {WS_URL}")
            while running:
                time.sleep(0.5)
            ws_conn.close()
        except Exception as e:
            print(f"[WS] Reconnecting... ({e})")
            time.sleep(2)

def send_command(cmd: str):
    global ws_conn, last_cmd
    if cmd == last_cmd:
        return
    try:
        if ws_conn:
            ws_conn.send(cmd)
            last_cmd = cmd
    except Exception as e:
        print(f"[WS] Send error: {e}")

def stream_thread():
    global latest_frame, running
    cap = cv2.VideoCapture(STREAM_URL)

    if not cap.isOpened():
        print(f"[CAM] Cannot open stream at {STREAM_URL}")
        running = False
        return

    print("[CAM] Stream opened")

    while running:
        ret, frame = cap.read()
        if ret:
            with frame_lock:
                latest_frame = frame

    cap.release()

BaseOptions = python.BaseOptions
HandLandmarker = vision.HandLandmarker
HandLandmarkerOptions = vision.HandLandmarkerOptions
VisionRunningMode = vision.RunningMode

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path="hand_landmarker.task"),
    running_mode=VisionRunningMode.VIDEO,
    num_hands=1,
    min_hand_detection_confidence=0.7,
    min_tracking_confidence=0.6,
)

landmarker = HandLandmarker.create_from_options(options)

def main():
    global running

    t_stream = threading.Thread(target=stream_thread, daemon=True)
    t_ws     = threading.Thread(target=ws_thread, daemon=True)

    t_stream.start()
    t_ws.start()

    time.sleep(1.5)

    frame_interval = 1.0 / args.fps
    next_tick = time.time()

    print("[MAIN] Starting hand tracking. Press Q to quit.")

    while running:
        now = time.time()
        if now < next_tick:
            time.sleep(0.005)
            continue
        next_tick = now + frame_interval

        with frame_lock:
            frame = latest_frame.copy() if latest_frame is not None else None

        if frame is None:
            continue

        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # ── MediaPipe processing ──
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        timestamp_ms = int(time.time() * 1000)

        result = landmarker.detect_for_video(mp_image, timestamp_ms)

        overlay = frame.copy()

        if result.hand_landmarks:
            hand_lm = result.hand_landmarks[0]

            # Convert to old-style landmark objects (for your IK function)
            class LM:
                def __init__(self, x, y, z):
                    self.x = x
                    self.y = y
                    self.z = z

            landmarks = [LM(l.x, l.y, l.z) for l in hand_lm]

            # Draw landmarks (manual)
            for lm in hand_lm:
                cx, cy = int(lm.x * w), int(lm.y * h)
                cv2.circle(overlay, (cx, cy), 3, (0, 255, 0), -1)

            angles = hand_to_arm(landmarks, w, h)
            cmd    = angles_to_cmd(angles)
            send_command(cmd)

            # HUD
            grip_label = "GRAB" if angles["grip"] else "OPEN"
            grip_color = (0, 60, 255) if angles["grip"] else (60, 220, 60)

            cv2.putText(overlay,
                        f"Base:{angles['base']}  J1:{angles['j1']}  J2:{angles['j2']}",
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (200, 200, 200), 1)

            cv2.putText(overlay,
                        f"J3:{angles['j3']}  J4:{angles['j4']}  Grip:{grip_label}",
                        (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        grip_color, 1)

        else:
            cv2.putText(overlay,
                        "No hand detected",
                        (8, 28),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (80, 80, 255),
                        1)

        cv2.imshow("Robotic Arm — Hand Control", overlay)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            running = False
            break

    cv2.destroyAllWindows()
    print("[MAIN] Exited.")

if __name__ == "__main__":
    main()
