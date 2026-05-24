#!/usr/bin/env python3
"""
Move the Piper arm to an XYZ position and hold until Ctrl+C.

Usage:
  python3 goto.py 250 0 300
  python3 goto.py 250 0 300 --gripper 50
  python3 goto.py 250 0 300 --rx 0 --ry 85 --rz 0
  python3 goto.py 250 0 300 --speed 30
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from can_setup import ensure_can
from piper_ik import PiperIK

MIN_Z = 80


def main():
    parser = argparse.ArgumentParser(description="Move arm to XYZ position and hold")
    parser.add_argument("x", type=float, help="X in mm")
    parser.add_argument("y", type=float, help="Y in mm")
    parser.add_argument("z", type=float, help="Z in mm")
    parser.add_argument("--rx", type=float, default=None, help="Rotation X in degrees")
    parser.add_argument("--ry", type=float, default=None, help="Rotation Y in degrees")
    parser.add_argument("--rz", type=float, default=None, help="Rotation Z in degrees")
    parser.add_argument("--gripper", type=float, default=0.0, help="Gripper opening in mm (default: 0)")
    parser.add_argument("--speed", type=int, default=30, help="Arm speed %% (default: 30)")
    parser.add_argument("--can", default="can0", help="CAN channel (default: can0)")
    args = parser.parse_args()

    if args.z < MIN_Z:
        print("ERROR: Z={:.0f} is below minimum safe height ({}mm). Aborting.".format(args.z, MIN_Z))
        sys.exit(1)

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
    if fk_result[2] < MIN_Z:
        print("ERROR: FK Z={:.0f} below minimum safe height ({}mm). Aborting.".format(fk_result[2], MIN_Z))
        sys.exit(1)

    print("Target:  ({:.0f}, {:.0f}, {:.0f})".format(args.x, args.y, args.z))
    print("FK check: ({:.1f}, {:.1f}, {:.1f})".format(fk_result[0], fk_result[1], fk_result[2]))
    degs = [math.degrees(j) for j in joints]
    print("Joints:  [{:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}] deg".format(*degs))

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

    millideg = [int(math.degrees(j) * 1000) for j in joints]
    gripper_units = int(args.gripper * 1000)

    try:
        print("\nMoving to target... (Ctrl+C to stop)")
        while True:
            piper.MotionCtrl_2(0x01, 0x01, args.speed, 0x00)
            piper.JointCtrl(*millideg)
            piper.GripperCtrl(gripper_units, 1000, 0x01, 0)
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        print("Returning to zero...")
        for _ in range(400):
            piper.MotionCtrl_2(0x01, 0x01, args.speed, 0x00)
            piper.JointCtrl(0, 0, 0, 0, 0, 0)
            piper.GripperCtrl(0, 1000, 0x01, 0)
            time.sleep(0.005)
        piper.DisablePiper()
        print("Done.")


if __name__ == "__main__":
    main()
