#!/usr/bin/env python3
"""
Coordinate Server — Receives XYZ positions via TCP and moves the Piper arm.

A simulator (or any client) connects via TCP and streams newline-delimited JSON
with x, y, z coordinates in millimeters. The server uses the Piper SDK's built-in
inverse kinematics (EndPoseCtrl) to move the end effector to that position.

Protocol:
  - Connect to TCP port 5555 (configurable)
  - Send one JSON object per line: {"x": 250.0, "y": 0.0, "z": 300.0}
  - Coordinates are in millimeters
  - Gripper stays closed, orientation is fixed (pointing down)

Usage:
  python3 coord_server.py                # Full mode (arm connected)
  python3 coord_server.py --no-arm       # Debug mode (print only, no robot)
  python3 coord_server.py --speed 20     # Slow speed for testing
"""

import argparse
import json
import socket
import time
import sys


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


# Approximate workspace bounds for the Piper arm (mm)
WORKSPACE_MIN = [-400, -400, 0]
WORKSPACE_MAX = [400, 400, 500]


def send_position(piper, x_mm, y_mm, z_mm, speed):
    """Send Cartesian position to the Piper arm via EndPoseCtrl."""
    # Clamp to workspace bounds
    x_mm = clamp(x_mm, WORKSPACE_MIN[0], WORKSPACE_MAX[0])
    y_mm = clamp(y_mm, WORKSPACE_MIN[1], WORKSPACE_MAX[1])
    z_mm = clamp(z_mm, WORKSPACE_MIN[2], WORKSPACE_MAX[2])

    # Convert mm to 0.001mm (SDK units)
    x_u = int(x_mm * 1000)
    y_u = int(y_mm * 1000)
    z_u = int(z_mm * 1000)

    # Fixed orientation (gripper pointing down): RX=0, RY=0, RZ=0
    piper.MotionCtrl_2(0x01, 0x00, speed, 0x00)
    piper.EndPoseCtrl(x_u, y_u, z_u, 0, 85_000, 0)
    piper.GripperCtrl(0, 1000, 0x01, 0)


def run_server(port, piper, speed, no_arm):
    """Run the TCP server, accepting one client at a time."""
    limiter = PositionLimiter(max_step_mm=5.0)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", port))
    server.listen(1)
    print(f"\nListening on port {port} (Ctrl+C to quit)...")

    try:
        while True:
            print("Waiting for client connection...")
            conn, addr = server.accept()
            print(f"Client connected: {addr}")

            buffer = ""
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

                        # Apply velocity limiting
                        x, y, z = limiter.limit(x, y, z)

                        if no_arm:
                            print(f"  Position: x={x:.1f} y={y:.1f} z={z:.1f} mm", end="\r")
                        else:
                            send_position(piper, x, y, z, speed)
                            print(f"  Sent: x={x:.1f} y={y:.1f} z={z:.1f} mm", end="\r")

            except ConnectionResetError:
                pass

            print(f"\nClient {addr} disconnected. Returning to home...")
            if not no_arm:
                send_position(piper, 0, 0, 0, speed)
            else:
                print("  Home: x=0.0 y=0.0 z=0.0 mm")
            limiter.prev = None
            conn.close()

    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        server.close()


def main():
    parser = argparse.ArgumentParser(description="TCP Coordinate Server → Piper Arm")
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
            print("Disabling arm...")
            piper.DisablePiper()
            print("Done.")


if __name__ == "__main__":
    main()
