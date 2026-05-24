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
import time
import sys


def send_coord(sock, x, y, z):
    """Send a single coordinate as newline-delimited JSON."""
    msg = json.dumps({"x": x, "y": y, "z": z}) + "\n"
    sock.sendall(msg.encode("utf-8"))


def return_to_home(sock, hz=30):
    """Smoothly send the arm back to 0, 0, 0."""
    print("\nReturning to home (0, 0, 0)...")
    send_coord(sock, 0.0, 0.0, 0.0)
    time.sleep(1.0 / hz)
    print("Home reached.")


def run_circle(sock, hz=30):
    """Send coordinates tracing a circle in the XZ plane."""
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
            send_coord(sock, x, y, z)
            print(f"  x={x:.1f} y={y:.1f} z={z:.1f}", end="\r")
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
    print("Enter coordinates as 'x y z' (Ctrl+C to quit):")
    try:
        while True:
            raw = input("  > ")
            parts = raw.strip().split()
            if len(parts) != 3:
                print("    Expected: x y z (e.g., 250 0 300)")
                continue
            try:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            except ValueError:
                print("    Invalid numbers.")
                continue
            send_coord(sock, x, y, z)
            print(f"    Sent: x={x:.1f} y={y:.1f} z={z:.1f}")
    except (KeyboardInterrupt, EOFError):
        pass

    return_to_home(sock)


def main():
    parser = argparse.ArgumentParser(description="Test client for coord_server")
    parser.add_argument("--host", default="127.0.0.1", help="Server host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=5555, help="Server port (default: 5555)")
    parser.add_argument("--hz", type=int, default=30, help="Send rate in Hz (default: 30)")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--circle", action="store_true", help="Circular motion pattern")
    mode.add_argument("--line", action="store_true", help="Linear back-and-forth pattern")
    mode.add_argument("--manual", action="store_true", help="Type coordinates manually")
    args = parser.parse_args()

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
