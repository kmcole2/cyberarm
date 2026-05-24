#!/usr/bin/env python3
"""
Coordinate Server — Receives XYZ positions via TCP and moves the Piper arm.

Uses our own IK solver (piper_ik.py) to convert Cartesian coordinates to
joint angles, then sends them via JointCtrl. This bypasses the firmware's
EndPoseCtrl which has silent failure modes.

Ports:
  - Coord port (default 5555): streams XYZ movement commands
  - Status port (default 5556): request/response for arm state queries

Protocol (coord port):
  - Send one JSON object per line: {"x": 250.0, "y": 0.0, "z": 300.0}
  - Coordinates are in millimeters

Protocol (status port):
  - Send: getPiperStatus\n
  - Receive: JSON with joints, end pose, arm status, gripper

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
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from can_setup import ensure_can
from piper_ik import PiperIK


def init_arm(can_channel):
    """Initialize Piper arm on CAN bus."""
    ensure_can(can_channel)
    from piper_sdk import C_PiperInterface_V2

    piper = C_PiperInterface_V2(can_channel)
    piper.ConnectPort()
    time.sleep(0.025)

    print(f"Piper firmware: {piper.GetPiperFirmwareVersion()}")
    print("Enabling arm...")
    while not piper.EnablePiper():
        time.sleep(0.01)
    print("Arm enabled.")

    # Initialize gripper: clear errors then enable
    piper.GripperCtrl(0, 1000, 0x02, 0)
    time.sleep(0.1)
    piper.GripperCtrl(0, 1000, 0x01, 0)
    time.sleep(0.1)
    print("Gripper initialized.")
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


def send_joints(piper, joints_rad, speed, gripper_mm=0.0):
    """Send joint angles and gripper command to the Piper arm."""
    millideg = [int(math.degrees(j) * 1000) for j in joints_rad]
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(*millideg)
    gripper_units = int(gripper_mm * 1000)
    piper.GripperCtrl(gripper_units, 1000, 0x01, 0)


def get_piper_status(piper):
    """Read current arm state and return as a dict."""
    joint_msg = piper.GetArmJointMsgs()
    js = joint_msg.joint_state
    joints_deg = [
        js.joint_1 / 1000.0,
        js.joint_2 / 1000.0,
        js.joint_3 / 1000.0,
        js.joint_4 / 1000.0,
        js.joint_5 / 1000.0,
        js.joint_6 / 1000.0,
    ]

    end_msg = piper.GetArmEndPoseMsgs()
    ep = end_msg.end_pose
    end_pose = {
        "x": ep.X_axis / 1000.0,
        "y": ep.Y_axis / 1000.0,
        "z": ep.Z_axis / 1000.0,
        "rx": ep.RX_axis / 1000.0,
        "ry": ep.RY_axis / 1000.0,
        "rz": ep.RZ_axis / 1000.0,
    }

    status_msg = piper.GetArmStatus()
    arm_st = status_msg.arm_status
    arm_status = {
        "enabled": arm_st.arm_status == 2,
        "motion_status": arm_st.motion_status,
        "err_code": arm_st.err_status,
    }

    gripper_msg = piper.GetArmGripperMsgs()
    gp = gripper_msg.gripper_state
    gripper = {
        "angle_mm": gp.grippers_angle / 1000.0,
        "effort_nm": gp.grippers_effort / 1000.0,
    }

    return {
        "joints_deg": joints_deg,
        "end_pose_mm": end_pose,
        "arm_status": arm_status,
        "gripper": gripper,
        "timestamp": time.time(),
    }


def get_mock_status():
    """Return zeroed status for --no-arm mode."""
    return {
        "joints_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "end_pose_mm": {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0},
        "arm_status": {"enabled": False, "motion_status": 0, "err_code": 0},
        "gripper": {"angle_mm": 0.0, "effort_nm": 0.0},
        "timestamp": time.time(),
    }


def run_status_server(port, piper, no_arm):
    """TCP server for status queries. Runs in a daemon thread."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", port))
    server.listen(1)
    print(f"Status server listening on port {port}")

    while True:
        try:
            conn, addr = server.accept()
        except OSError:
            break

        buffer = ""
        try:
            while True:
                data = conn.recv(4096)
                if not data:
                    break

                buffer += data.decode("utf-8")

                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    cmd = line.strip()
                    if not cmd:
                        continue

                    if cmd == "getPiperStatus":
                        if no_arm:
                            status = get_mock_status()
                        else:
                            status = get_piper_status(piper)
                        response = json.dumps(status) + "\n"
                    else:
                        response = json.dumps({"error": "unknown command"}) + "\n"

                    conn.sendall(response.encode("utf-8"))

        except (ConnectionResetError, BrokenPipeError):
            pass
        finally:
            conn.close()


def run_coord_server(port, piper, speed, no_arm):
    """Run the TCP server for coordinate commands, accepting one client at a time."""
    limiter = PositionLimiter(max_step_mm=5.0)
    ik = PiperIK()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", port))
    server.listen(1)
    print(f"Coord server listening on port {port}")
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

                        gripper_mm = float(msg.get("gripper", 0.0))

                        print("  recv: x={:.1f} y={:.1f} z={:.1f} grip={:.1f}".format(
                            x, y, z, gripper_mm))

                        x, y, z = limiter.limit(x, y, z)

                        x = clamp(x, WORKSPACE_MIN[0], WORKSPACE_MAX[0])
                        y = clamp(y, WORKSPACE_MIN[1], WORKSPACE_MAX[1])
                        z = clamp(z, WORKSPACE_MIN[2], WORKSPACE_MAX[2])

                        joints, converged = ik.solve_position_only([x, y, z])

                        if not converged:
                            ik_failures += 1
                            if ik_failures <= 5:
                                print("  IK did not converge for ({:.1f}, {:.1f}, {:.1f})".format(x, y, z))
                            continue

                        if no_arm:
                            degs = ["{:.0f}".format(math.degrees(j)) for j in joints]
                            print("  xyz=({:.1f},{:.1f},{:.1f}) grip={:.1f} -> joints={}".format(
                                x, y, z, gripper_mm, degs), end="\r")
                        else:
                            send_joints(piper, joints, speed, gripper_mm)
                            print("  xyz=({:.1f},{:.1f},{:.1f}) grip={:.1f} -> sent".format(
                                x, y, z, gripper_mm), end="\r")

            except ConnectionResetError:
                pass

            print(f"\nClient {addr} disconnected (IK failures: {ik_failures}). Returning home...")
            if not no_arm:
                for _ in range(300):
                    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
                    piper.JointCtrl(0, 0, 0, 0, 0, 0)
                    piper.GripperCtrl(0, 1000, 0x01, 0)
                    time.sleep(0.005)
            limiter.prev = None
            conn.close()

    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        server.close()


def main():
    parser = argparse.ArgumentParser(description="TCP Coordinate Server → Piper Arm (Custom IK)")
    parser.add_argument("--port", type=int, default=5555, help="Coord TCP port (default: 5555)")
    parser.add_argument("--status-port", type=int, default=5556, help="Status TCP port (default: 5556)")
    parser.add_argument("--can", default="can0", help="CAN channel (default: can0)")
    parser.add_argument("--speed", type=int, default=50, help="Arm speed %% (default: 50)")
    parser.add_argument("--no-arm", action="store_true", help="Debug mode (no robot)")
    args = parser.parse_args()

    piper = None
    if not args.no_arm:
        piper = init_arm(args.can)

    status_thread = threading.Thread(
        target=run_status_server,
        args=(args.status_port, piper, args.no_arm),
        daemon=True,
    )
    status_thread.start()

    try:
        run_coord_server(args.port, piper, args.speed, args.no_arm)
    finally:
        if piper:
            print("Returning to home...")
            for _ in range(300):
                piper.MotionCtrl_2(0x01, 0x01, args.speed, 0x00)
                piper.JointCtrl(0, 0, 0, 0, 0, 0)
                piper.GripperCtrl(0, 1000, 0x01, 0)
                time.sleep(0.005)
            print("Disabling arm...")
            piper.DisablePiper()
            print("Done.")


if __name__ == "__main__":
    main()
