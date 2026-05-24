#!/usr/bin/env python3
"""
Move the Piper arm to an XYZ position.

Usage:
  python3 goto.py 250 0 300
  python3 goto.py 250 0 300 --gripper 50
  python3 goto.py 250 0 300 --rx 0 --ry 85 --rz 0
  python3 goto.py 250 0 300 --gripper 50 --speed 30
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from can_setup import ensure_can
from piper_ik import PiperIK

SAFE_JOINTS_MDEG = (0, 40_000, -60_000, 0, 0, 0)
MIN_Z = 80


def send_joints(piper, joints_rad, speed, gripper_mm=0.0, duration=2.0):
    millideg = [int(math.degrees(j) * 1000) for j in joints_rad]
    gripper_units = int(gripper_mm * 1000)
    steps = int(duration / 0.005)
    for step in range(steps):
        piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
        piper.JointCtrl(*millideg)
        piper.GripperCtrl(gripper_units, 1000, 0x01, 0)
        if step % 40 == 0:
            ep = piper.GetArmEndPoseMsgs().end_pose
            print("  pos=({:.0f},{:.0f},{:.0f}) gripper={:.0f}mm".format(
                ep.X_axis / 1000.0, ep.Y_axis / 1000.0, ep.Z_axis / 1000.0,
                gripper_mm), end="\r")
        time.sleep(0.005)
    print()


def send_millideg(piper, j1, j2, j3, j4, j5, j6, speed, gripper_mm=0.0, duration=2.0):
    gripper_units = int(gripper_mm * 1000)
    steps = int(duration / 0.005)
    for _ in range(steps):
        piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
        piper.JointCtrl(j1, j2, j3, j4, j5, j6)
        piper.GripperCtrl(gripper_units, 1000, 0x01, 0)
        time.sleep(0.005)


def main():
    parser = argparse.ArgumentParser(description="Move arm to XYZ position")
    parser.add_argument("x", type=float, help="X in mm")
    parser.add_argument("y", type=float, help="Y in mm")
    parser.add_argument("z", type=float, help="Z in mm")
    parser.add_argument("--rx", type=float, default=None, help="Rotation X in degrees")
    parser.add_argument("--ry", type=float, default=None, help="Rotation Y in degrees")
    parser.add_argument("--rz", type=float, default=None, help="Rotation Z in degrees")
    parser.add_argument("--gripper", type=float, default=0.0, help="Gripper opening in mm (default: 0)")
    parser.add_argument("--speed", type=int, default=30, help="Arm speed %% (default: 30)")
    parser.add_argument("--hold", type=float, default=3.0, help="Hold time in seconds (default: 3)")
    parser.add_argument("--can", default="can0", help="CAN channel (default: can0)")
    args = parser.parse_args()

    if args.z < MIN_Z:
        print("ERROR: Z={:.0f} is below minimum safe height ({}mm). Aborting.".format(args.z, MIN_Z))
        sys.exit(1)

    ensure_can(args.can)

    from piper_sdk import C_PiperInterface_V2
    piper = C_PiperInterface_V2(args.can)
    piper.ConnectPort()
    time.sleep(0.5)

    print("Enabling arm...")
    while not piper.EnablePiper():
        time.sleep(0.01)
    print("Arm enabled!")

    piper.GripperCtrl(0, 1000, 0x02, 0)
    piper.GripperCtrl(0, 1000, 0x01, 0)
    time.sleep(0.5)

    ik = PiperIK()

    has_rotation = args.rx is not None or args.ry is not None or args.rz is not None
    if has_rotation:
        rx = args.rx if args.rx is not None else 0.0
        ry = args.ry if args.ry is not None else 85.0
        rz = args.rz if args.rz is not None else 0.0
        joints, converged = ik.solve([args.x, args.y, args.z], [rx, ry, rz])
    else:
        joints, converged = ik.solve_position_only([args.x, args.y, args.z])

    if not converged:
        print("ERROR: IK did not converge for ({:.0f}, {:.0f}, {:.0f}). Position may be out of reach.".format(
            args.x, args.y, args.z))
        sys.exit(1)

    fk_result = ik.forward(joints)
    print("Target:  ({:.0f}, {:.0f}, {:.0f})".format(args.x, args.y, args.z))
    print("FK check: ({:.1f}, {:.1f}, {:.1f})".format(fk_result[0], fk_result[1], fk_result[2]))
    degs = [math.degrees(j) for j in joints]
    print("Joints:  [{:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}] deg".format(*degs))

    if fk_result[2] < MIN_Z:
        print("ERROR: FK Z={:.0f} below minimum safe height ({}mm). Aborting.".format(fk_result[2], MIN_Z))
        sys.exit(1)

    try:
        print("\nRaising arm to safe pose...")
        send_millideg(piper, *SAFE_JOINTS_MDEG, args.speed, duration=2.0)

        print("Moving to target ({:.0f}, {:.0f}, {:.0f})...".format(args.x, args.y, args.z))
        send_joints(piper, joints, args.speed, args.gripper, duration=3.0)

        print("Holding for {:.0f}s (Ctrl+C to stop)...".format(args.hold))
        t0 = time.time()
        gripper_units = int(args.gripper * 1000)
        millideg = [int(math.degrees(j) * 1000) for j in joints]
        while time.time() - t0 < args.hold:
            piper.MotionCtrl_2(0x01, 0x01, args.speed, 0x00)
            piper.JointCtrl(*millideg)
            piper.GripperCtrl(gripper_units, 1000, 0x01, 0)
            time.sleep(0.005)

        ep = piper.GetArmEndPoseMsgs().end_pose
        print("Final position: ({:.1f}, {:.1f}, {:.1f})".format(
            ep.X_axis / 1000.0, ep.Y_axis / 1000.0, ep.Z_axis / 1000.0))

    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    finally:
        print("\nReturning to safe pose...")
        send_millideg(piper, *SAFE_JOINTS_MDEG, args.speed, duration=2.0)
        print("Returning to zero...")
        send_millideg(piper, 0, 0, 0, 0, 0, 0, args.speed, duration=2.0)
        print("Disabling arm...")
        piper.DisablePiper()
        print("Done.")


if __name__ == "__main__":
    main()
