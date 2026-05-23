#!/usr/bin/env python3
"""
Arm Mirror — Real-time human arm mirroring to Piper robot.

Uses Orbbec camera + MediaPipe Pose to detect arm joints,
calculates angles, and sends them to the Piper arm via CAN bus.

Usage:
  python3 arm_mirror.py                  # Full mode (camera + arm)
  python3 arm_mirror.py --no-arm         # Vision-only debug (no robot)
  python3 arm_mirror.py --speed 20       # Slow speed for testing
  python3 arm_mirror.py --arm left       # Track left arm instead
"""

import argparse
import sys
import os
import time

import cv2
import numpy as np
import mediapipe as mp

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from angle_utils import (
    AngleSmoother,
    VelocityLimiter,
    check_visibility,
    clamp_to_limits,
    degrees_to_millidegrees,
    landmarks_to_piper_joints,
)

from pyorbbecsdk import (
    Config,
    Context,
    OBLogLevel,
    OBSensorType,
    Pipeline,
)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sdks", "pyorbbecsdk", "examples"))
from utils import frame_to_bgr_image


def init_camera():
    """Initialize Orbbec camera with color stream."""
    Context.set_logger_to_console(OBLogLevel.WARNING)
    ctx = Context()
    device_list = ctx.query_devices()
    if device_list.get_count() == 0:
        print("No Orbbec camera found. Check USB connection.")
        sys.exit(1)

    device = device_list.get_device_by_index(0)
    info = device.get_device_info()
    print(f"Camera: {info.get_name()} (SN: {info.get_serial_number()})")

    pipeline = Pipeline(device)
    config = Config()
    profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    color_profile = profile_list.get_default_video_stream_profile()
    config.enable_stream(color_profile)
    print(f"Color stream: {color_profile.get_width()}x{color_profile.get_height()} @ {color_profile.get_fps()}fps")

    pipeline.start(config)
    return pipeline


def init_arm(can_channel):
    """Initialize Piper arm on CAN bus."""
    from piper_sdk import C_PiperInterface_V2

    piper = C_PiperInterface_V2(can_channel)
    piper.ConnectPort()
    time.sleep(0.025)

    print(f"Piper firmware: {piper.GetPiperFirmwareVersion()}")
    print("Enabling arm...")
    while not piper.EnablePiper():
        time.sleep(0.01)
    print("Arm enabled.")
    return piper


def send_to_arm(piper, millideg_angles, speed):
    """Send joint angles to the Piper arm."""
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(*millideg_angles)


def draw_overlay(frame, angles, tracking, fps):
    """Draw joint angle info and status on the frame."""
    h, w = frame.shape[:2]

    status_color = (0, 255, 0) if tracking else (0, 0, 255)
    status_text = "TRACKING" if tracking else "LOST"
    cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
    cv2.putText(frame, f"{fps:.0f} FPS", (w - 120, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    if angles:
        labels = ["J1:base", "J2:shldr", "J3:elbow", "J4:roll", "J5:pitch", "J6:yaw"]
        for i, (label, deg) in enumerate(zip(labels, angles)):
            text = f"{label} {deg:6.1f} deg"
            cv2.putText(frame, text, (10, 70 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 200), 1)


def main():
    parser = argparse.ArgumentParser(description="Arm Mirror: camera pose → Piper robot")
    parser.add_argument("--can", default="can0", help="CAN channel (default: can0)")
    parser.add_argument("--speed", type=int, default=50, help="Arm speed %% (default: 50)")
    parser.add_argument("--alpha", type=float, default=0.3, help="Smoothing factor 0-1 (default: 0.3)")
    parser.add_argument("--max-step", type=float, default=5.0, help="Max degrees per frame (default: 5.0)")
    parser.add_argument("--arm", choices=["left", "right"], default="right", help="Which arm to track")
    parser.add_argument("--no-arm", action="store_true", help="Vision-only mode (no robot)")
    args = parser.parse_args()

    # --- Init MediaPipe ---
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils
    pose = mp_pose.Pose(
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5,
        model_complexity=1,
    )

    # --- Init Camera ---
    print("Initializing camera...")
    pipeline = init_camera()

    # --- Init Arm ---
    piper = None
    if not args.no_arm:
        print("Initializing arm...")
        piper = init_arm(args.can)

    # --- Processing state ---
    smoother = AngleSmoother(alpha=args.alpha)
    limiter = VelocityLimiter(max_step_deg=args.max_step)
    frames_lost = 0
    max_lost_frames = 30  # ~1 second at 30fps
    current_angles = None
    prev_time = time.time()

    print("\nRunning arm mirror (press Q or ESC to quit)...\n")

    try:
        while True:
            frameset = pipeline.wait_for_frames(100)
            if frameset is None:
                continue

            color_frame = frameset.get_color_frame()
            if color_frame is None:
                continue

            bgr_image = frame_to_bgr_image(color_frame)
            if bgr_image is None:
                continue

            # MediaPipe expects RGB
            rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
            results = pose.process(rgb_image)

            tracking = False

            if results.pose_world_landmarks:
                wl = results.pose_world_landmarks.landmark

                if check_visibility(wl, side=args.arm):
                    raw_angles = landmarks_to_piper_joints(wl, side=args.arm)
                    smoothed = smoother.smooth(raw_angles)
                    limited = limiter.limit(smoothed)
                    clamped = clamp_to_limits(limited)
                    current_angles = clamped
                    frames_lost = 0
                    tracking = True

                    if piper:
                        millideg = degrees_to_millidegrees(clamped)
                        send_to_arm(piper, millideg, args.speed)

            if not tracking:
                frames_lost += 1
                if frames_lost >= max_lost_frames and piper and current_angles:
                    # Return to home position
                    send_to_arm(piper, [0, 0, 0, 0, 0, 0], args.speed)
                    current_angles = None

            # Draw landmarks on image
            if results.pose_landmarks:
                mp_drawing.draw_landmarks(
                    bgr_image,
                    results.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS,
                )

            # FPS calculation
            now = time.time()
            fps = 1.0 / (now - prev_time + 1e-8)
            prev_time = now

            draw_overlay(bgr_image, current_angles, tracking, fps)
            cv2.imshow("Arm Mirror", bgr_image)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break

    except KeyboardInterrupt:
        print("\nInterrupted.")

    # --- Cleanup ---
    print("Shutting down...")
    if piper:
        print("Returning arm to home position...")
        for _ in range(300):
            send_to_arm(piper, [0, 0, 0, 0, 0, 0], args.speed)
            time.sleep(0.005)
        piper.DisablePiper()
        print("Arm disabled.")

    pipeline.stop()
    cv2.destroyAllWindows()
    pose.close()
    print("Done.")


if __name__ == "__main__":
    main()
