#!/usr/bin/env python3
"""
Test Client — Sends sample XYZ coordinates to the coord_server.

Use this to test the server without a real simulator.

Usage:
  python3 coord_client.py --circle       # Circular motion in XZ plane
  python3 coord_client.py --line         # Back-and-forth along X axis
  python3 coord_client.py --manual       # Type coordinates interactively
"""

import argparse
import json
import math
import socket
import threading
import time
import sys


def send_coord(sock, x, y, z, rx=None, ry=None, rz=None, gripper=None):
    """Send a coordinate as newline-delimited JSON. Rotation/gripper are optional."""
    msg = {"x": x, "y": y, "z": z}
    if rx is not None:
        msg["rx"] = rx
    if ry is not None:
        msg["ry"] = ry
    if rz is not None:
        msg["rz"] = rz
    if gripper is not None:
        msg["gripper"] = gripper
    sock.sendall((json.dumps(msg) + "\n").encode("utf-8"))


def return_to_home(sock, hz=30):
    """Smoothly send the arm back to 0, 0, 0."""
    print("\nReturning to home (0, 0, 0)...")
    send_coord(sock, 0.0, 0.0, 0.0)
    time.sleep(1.0 / hz)
    print("Home reached.")


def run_circle(sock, hz=30):
    """Send coordinates tracing a circle in the XZ plane with gripper cycling."""
    print("Sending circle pattern (Ctrl+C to stop)...")
    center_x, center_y, center_z = 250.0, 0.0, 250.0
    radius = 50.0
    t = 0.0
    period = 4.0  # seconds per revolution

    try:
        while True:
            x = center_x + radius * math.cos(2 * math.pi * t / period)
            z = center_z + radius * math.sin(2 * math.pi * t / period)
            y = center_y
            # Cycle gripper: open at top, closed at bottom
            gripper = 35.0 * (1 + math.sin(2 * math.pi * t / period)) / 2.0
            send_coord(sock, x, y, z, ry=85.0, gripper=gripper)
            print(f"  x={x:.1f} y={y:.1f} z={z:.1f} grip={gripper:.1f}mm", end="\r")
            time.sleep(1.0 / hz)
            t += 1.0 / hz
    except KeyboardInterrupt:
        pass

    return_to_home(sock, hz)


def run_line(sock, hz=30):
    """Send coordinates moving back and forth along the X axis."""
    print("Sending line pattern (Ctrl+C to stop)...")
    y, z = 0.0, 250.0
    x_min, x_max = 150.0, 350.0
    speed_mm_per_sec = 50.0
    x = x_min
    direction = 1

    try:
        while True:
            x += direction * speed_mm_per_sec / hz
            if x >= x_max:
                x = x_max
                direction = -1
            elif x <= x_min:
                x = x_min
                direction = 1
            send_coord(sock, x, y, z)
            print(f"  x={x:.1f} y={y:.1f} z={z:.1f}", end="\r")
            time.sleep(1.0 / hz)
    except KeyboardInterrupt:
        pass

    return_to_home(sock, hz)


def run_manual(sock):
    """Interactively type coordinates."""
    print("Enter coordinates as 'x y z [rx ry rz] [gripper]' (Ctrl+C to quit):")
    print("  Examples: 250 0 300")
    print("           250 0 300 0 85 0")
    print("           250 0 300 0 85 0 35.0")
    try:
        while True:
            raw = input("  > ")
            parts = raw.strip().split()
            if len(parts) < 3:
                print("    Expected at least: x y z")
                continue
            try:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                rx = float(parts[3]) if len(parts) > 3 else None
                ry = float(parts[4]) if len(parts) > 4 else None
                rz = float(parts[5]) if len(parts) > 5 else None
                gripper = float(parts[6]) if len(parts) > 6 else None
            except ValueError:
                print("    Invalid numbers.")
                continue
            send_coord(sock, x, y, z, rx=rx, ry=ry, rz=rz, gripper=gripper)
            print(f"    Sent: x={x:.1f} y={y:.1f} z={z:.1f}", end="")
            if rx is not None:
                print(f" rx={rx:.1f} ry={ry:.1f} rz={rz:.1f}", end="")
            if gripper is not None:
                print(f" gripper={gripper:.1f}mm", end="")
            print()
    except (KeyboardInterrupt, EOFError):
        pass

    return_to_home(sock)


def status_poller(host, status_port):
    """Background thread: queries getPiperStatus every second and prints it."""
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            sock.connect((host, status_port))
            while True:
                sock.sendall(b"getPiperStatus\n")
                data = sock.recv(4096).decode("utf-8").strip()
                if data:
                    status = json.loads(data)
                    joints = status["joints_deg"]
                    ep = status["end_pose_mm"]
                    arm = status["arm_status"]
                    print(
                        f"\n  [STATUS] joints=[{joints[0]:.1f},{joints[1]:.1f},{joints[2]:.1f},"
                        f"{joints[3]:.1f},{joints[4]:.1f},{joints[5]:.1f}] "
                        f"pos=({ep['x']:.1f},{ep['y']:.1f},{ep['z']:.1f}) "
                        f"enabled={arm['enabled']} err={arm['err_code']}"
                    )
                time.sleep(1.0)
        except (ConnectionRefusedError, OSError):
            time.sleep(1.0)
        finally:
            try:
                sock.close()
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(description="Test client for coord_server")
    parser.add_argument("--host", default="127.0.0.1", help="Server host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=5555, help="Server port (default: 5555)")
    parser.add_argument("--status-port", type=int, default=5556, help="Status port (default: 5556)")
    parser.add_argument("--hz", type=int, default=30, help="Send rate in Hz (default: 30)")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--circle", action="store_true", help="Circular motion pattern")
    mode.add_argument("--line", action="store_true", help="Linear back-and-forth pattern")
    mode.add_argument("--manual", action="store_true", help="Type coordinates manually")
    args = parser.parse_args()

    # Start status polling in background
    poller = threading.Thread(target=status_poller, args=(args.host, args.status_port), daemon=True)
    poller.start()

    print(f"Connecting to {args.host}:{args.port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((args.host, args.port))
    except ConnectionRefusedError:
        print(f"Cannot connect. Is the server running on port {args.port}?")
        sys.exit(1)
    print("Connected.")

    try:
        if args.circle:
            run_circle(sock, args.hz)
        elif args.line:
            run_line(sock, args.hz)
        elif args.manual:
            run_manual(sock)
    finally:
        sock.close()


if __name__ == "__main__":
    main()
