#!/usr/bin/env python3
"""
CAN bus setup for the Piper arm.

Checks if the CAN interface is up and activates it if needed.
Equivalent to running find_all_can_port.sh + can_activate.sh manually.

Requires: Linux with ethtool, can-utils, and sudo access.
"""

import subprocess
import sys


def _run(cmd, check=True):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, check=check)


def _check_dependency(name):
    result = _run(f"dpkg -l | grep -q {name}", check=False)
    if result.returncode != 0:
        print(f"Error: {name} not installed. Run: sudo apt install {name}")
        sys.exit(1)


def _get_can_interfaces():
    result = _run("ip -br link show type can", check=False)
    if result.returncode != 0 or not result.stdout.strip():
        return []
    interfaces = []
    for line in result.stdout.strip().split("\n"):
        parts = line.split()
        if parts:
            interfaces.append(parts[0])
    return interfaces


def _is_interface_up(iface):
    result = _run(f"ip link show {iface}", check=False)
    return "UP" in result.stdout


def _get_bitrate(iface):
    result = _run(f"ip -details link show {iface}", check=False)
    for token in result.stdout.split():
        if token.isdigit():
            idx = result.stdout.find("bitrate")
            if idx >= 0:
                parts = result.stdout[idx:].split()
                if len(parts) >= 2:
                    return int(parts[1])
    return 0


def ensure_can(can_name="can0", bitrate=1000000):
    """
    Ensure the CAN interface is up with the correct name and bitrate.

    Skips silently on non-Linux (macOS) so the script doesn't crash
    during development.
    """
    if sys.platform != "linux":
        print(f"[can_setup] Not Linux — skipping CAN setup (assuming {can_name} is managed externally)")
        return

    _check_dependency("ethtool")
    _check_dependency("can-utils")

    interfaces = _get_can_interfaces()

    if not interfaces:
        print("[can_setup] No CAN interfaces detected. Is the USB-CAN adapter plugged in?")
        sys.exit(1)

    if len(interfaces) > 1:
        print(f"[can_setup] Multiple CAN interfaces found: {interfaces}")
        print("  Specify USB address manually. See can_activate.sh for details.")
        sys.exit(1)

    iface = interfaces[0]
    print(f"[can_setup] Found CAN interface: {iface}")

    is_up = _is_interface_up(iface)
    current_bitrate = _get_bitrate(iface) if is_up else 0

    if is_up and current_bitrate == bitrate and iface == can_name:
        print(f"[can_setup] {can_name} already up at {bitrate} bps — no action needed")
        return

    if is_up and current_bitrate == bitrate and iface != can_name:
        print(f"[can_setup] Renaming {iface} → {can_name}")
        _run(f"sudo ip link set {iface} down")
        _run(f"sudo ip link set {iface} name {can_name}")
        _run(f"sudo ip link set {can_name} up")
        print(f"[can_setup] {can_name} is up")
        return

    print(f"[can_setup] Configuring {iface} → {can_name} at {bitrate} bps")
    _run(f"sudo ip link set {iface} down")
    _run(f"sudo ip link set {iface} type can bitrate {bitrate}")
    _run(f"sudo ip link set {iface} up")

    if iface != can_name:
        _run(f"sudo ip link set {iface} down")
        _run(f"sudo ip link set {iface} name {can_name}")
        _run(f"sudo ip link set {can_name} up")

    print(f"[can_setup] {can_name} is up at {bitrate} bps")


if __name__ == "__main__":
    ensure_can()
