#!/usr/bin/env python3
"""
Coordinate Server — Receives XYZ positions via TCP and moves the Piper arm.

Uses our own IK solver (piper_ik.py) to convert Cartesian coordinates to
joint angles, then sends them via JointCtrl. This bypasses the firmware's
EndPoseCtrl which has silent failure modes.

Protocol:
  - Connect to TCP port 5555 (configurable)
  - Send one JSON object per line: {"x": 250.0, "y": 0.0, "z": 300.0}
  - Coordinates are in millimeters
  - Gripper stays closed, orientation is fixed (pointing forward)

Usage:
  python3 coord_server.py                # Full mode (arm connected)
  python3 coord_server.py --no-arm       # Debug mode (print only, no robot)
  python3 coord_server.py --speed 20     # Slow speed for testing
"""

import argparse
import json
import math
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from piper_ik import PiperIK


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

    piper.GripperCtrl(0, 1000, 0x01, 0)
    print("Gripper closed.")
    return piper


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class PositionLimiter:
    """Limits per-step position change to prevent violent movements."""

    def __init__(self, max_step_mm=5.0):
        self.max_step = max_step_mm
        self.prev = None

    def limit(self, x, y, z):
        pos = [x, y, z]
        if self.prev is None:
            self.prev = pos
            return pos

        limited = []
        for i in range(3):
            delta = pos[i] - self.prev[i]
            clamped_delta = max(-self.max_step, min(self.max_step, delta))
            limited.append(self.prev[i] + clamped_delta)
        self.prev = limited
        return limited


WORKSPACE_MIN = [-400, -400, 0]
WORKSPACE_MAX = [400, 400, 500]


def send_joints(piper, joints_rad, speed):
    """Send joint angles to the Piper arm via JointCtrl."""
    millideg = [int(math.degrees(j) * 1000) for j in joints_rad]
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(*millideg)
    piper.GripperCtrl(0, 1000, 0x01, 0)


def run_server(port, piper, speed, no_arm):
    """Run the TCP server, accepting one client at a time."""
    limiter = PositionLimiter(max_step_mm=5.0)
    ik = PiperIK()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", port))
    server.listen(1)
    print(f"\nListening on port {port} (Ctrl+C to quit)...")
    print("Using custom IK → JointCtrl (bypassing EndPoseCtrl)")

    try:
        while True:
            print("Waiting for client connection...")
            conn, addr = server.accept()
            print(f"Client connected: {addr}")

            buffer = ""
            ik_failures = 0
            try:
                while True:
                    data = conn.recv(4096)
                    if not data:
                        break

                    buffer += data.decode("utf-8")

                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue

                        try:
                            msg = json.loads(line)
                            x = float(msg["x"])
                            y = float(msg["y"])
                            z = float(msg["z"])
                        except (json.JSONDecodeError, KeyError, ValueError) as e:
                            print(f"  Bad message: {line!r} ({e})")
                            continue

                        x, y, z = limiter.limit(x, y, z)

                        x = clamp(x, WORKSPACE_MIN[0], WORKSPACE_MAX[0])
                        y = clamp(y, WORKSPACE_MIN[1], WORKSPACE_MAX[1])
                        z = clamp(z, WORKSPACE_MIN[2], WORKSPACE_MAX[2])

                        joints, converged = ik.solve_position_only([x, y, z])

                        if not converged:
                            ik_failures += 1
                            if ik_failures <= 5:
                                print(f"  IK did not converge for ({x:.1f}, {y:.1f}, {z:.1f})")
                            continue

                        if no_arm:
                            degs = [f"{math.degrees(j):.0f}" for j in joints]
                            print(f"  xyz=({x:.1f},{y:.1f},{z:.1f}) → joints={degs}", end="\r")
                        else:
                            send_joints(piper, joints, speed)
                            print(f"  xyz=({x:.1f},{y:.1f},{z:.1f}) → sent", end="\r")

            except ConnectionResetError:
                pass

            print(f"\nClient {addr} disconnected (IK failures: {ik_failures}). Returning home...")
            if not no_arm:
                send_joints(piper, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], speed)
            limiter.prev = None
            conn.close()

    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        server.close()


def main():
    parser = argparse.ArgumentParser(description="TCP Coordinate Server → Piper Arm (Custom IK)")
    parser.add_argument("--port", type=int, default=5555, help="TCP port (default: 5555)")
    parser.add_argument("--can", default="can0", help="CAN channel (default: can0)")
    parser.add_argument("--speed", type=int, default=50, help="Arm speed %% (default: 50)")
    parser.add_argument("--no-arm", action="store_true", help="Debug mode (no robot)")
    args = parser.parse_args()

    piper = None
    if not args.no_arm:
        piper = init_arm(args.can)

    try:
        run_server(args.port, piper, args.speed, args.no_arm)
    finally:
        if piper:
            print("Returning to home...")
            for _ in range(300):
                piper.MotionCtrl_2(0x01, 0x01, args.speed, 0x00)
                piper.JointCtrl(0, 0, 0, 0, 0, 0)
                time.sleep(0.005)
            print("Disabling arm...")
            piper.DisablePiper()
            print("Done.")


if __name__ == "__main__":
    main()
