#!/usr/bin/env python3
"""
Physical workspace exploration using direct joint commands.

Sweeps J1 (base rotation) and J2/J3 (reach) to map out the workspace.
Safety: Y >= 0 and Z >= 80mm enforced via FK check before each move.
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from can_setup import ensure_can
from piper_ik import PiperIK

ensure_can()

from piper_sdk import C_PiperInterface_V2

SPEED = 30
MIN_Z = 80
MIN_Y = -5

piper = C_PiperInterface_V2("can0")
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

reached = []
skipped = []


def fk_pos(joints_deg):
    joints_rad = [math.radians(j) for j in joints_deg]
    result = ik.forward(joints_rad)
    return result[0], result[1], result[2]


def is_safe(j1_deg, j2_deg, j3_deg):
    x, y, z = fk_pos([j1_deg, j2_deg, j3_deg, 0, 0, 0])
    if y < MIN_Y:
        return False, x, y, z, "Y={:.0f} < {}".format(y, MIN_Y)
    if z < MIN_Z:
        return False, x, y, z, "Z={:.0f} < {}".format(z, MIN_Z)
    return True, x, y, z, ""


def send(j1, j2, j3, j4, j5, j6, duration=1.5):
    steps = int(duration / 0.005)
    for _ in range(steps):
        piper.MotionCtrl_2(0x01, 0x01, SPEED, 0x00)
        piper.JointCtrl(j1, j2, j3, j4, j5, j6)
        piper.GripperCtrl(0, 1000, 0x01, 0)
        time.sleep(0.005)


SAFE_POSE = (0, 40_000, -60_000, 0, 0, 0)


def go_safe(duration=2.0):
    print("  -> Moving to safe pose...")
    send(*SAFE_POSE, duration=duration)


def go_home(duration=2.0):
    print("  -> Returning home...")
    go_safe(duration)
    send(0, 0, 0, 0, 0, 0, duration)


def move_and_report(j1_deg, j2_deg, j3_deg, label=""):
    safe, x, y, z, reason = is_safe(j1_deg, j2_deg, j3_deg)
    if not safe:
        print("  J1={:4.0f} J2={:4.0f} J3={:4.0f} -> SKIP ({}) FK=({:.0f},{:.0f},{:.0f}) {}".format(
            j1_deg, j2_deg, j3_deg, reason, x, y, z, label))
        skipped.append({"joints": (j1_deg, j2_deg, j3_deg), "fk": (x, y, z), "reason": reason})
        return False

    print("  J1={:4.0f} J2={:4.0f} J3={:4.0f} -> FK({:.0f},{:.0f},{:.0f}) ".format(
        j1_deg, j2_deg, j3_deg, x, y, z), end="")
    sys.stdout.flush()

    send(int(j1_deg * 1000), int(j2_deg * 1000), int(j3_deg * 1000), 0, 0, 0, duration=1.5)

    ep = piper.GetArmEndPoseMsgs().end_pose
    ax, ay, az = ep.X_axis / 1000.0, ep.Y_axis / 1000.0, ep.Z_axis / 1000.0
    print("actual=({:.0f},{:.0f},{:.0f}) {}".format(ax, ay, az, label))
    reached.append({"joints": (j1_deg, j2_deg, j3_deg), "fk": (x, y, z), "actual": (ax, ay, az)})
    time.sleep(0.5)
    return True


try:
    print("\n=== Raising arm to safe starting position ===")
    go_safe(duration=2.0)
    time.sleep(1.0)

    print("\n=== Sweep J1 (base rotation) -150 to +150 ===")
    print("  J2=40, J3=-60 (arm raised, safe height)")
    for j1 in range(-150, 151, 15):
        move_and_report(j1, 40, -60)
    go_safe()

    print("\n=== Sweep J2 (shoulder) at J1=0 ===")
    go_safe()
    for j2 in range(0, 181, 15):
        move_and_report(0, j2, -60)
    go_safe()

    print("\n=== Sweep J3 (elbow) at J1=0, J2=40 ===")
    go_safe()
    for j3 in range(0, -171, -15):
        move_and_report(0, 40, j3)
    go_safe()

    print("\n=== Max +Y: J1=90 with various extensions ===")
    go_safe()
    for j2, j3 in [(20, -30), (30, -45), (40, -60), (50, -80), (60, -90)]:
        move_and_report(90, j2, j3, label="<-- +Y reach")
    go_safe()

    print("\n=== Max +X: J1=0 with various extensions ===")
    go_safe()
    for j2, j3 in [(20, -30), (30, -45), (40, -60), (50, -80), (60, -90)]:
        move_and_report(0, j2, j3, label="<-- +X reach")
    go_safe()

    print("\n=== Max -X: J1=180 with various extensions ===")
    go_safe()
    for j2, j3 in [(20, -30), (30, -45), (40, -60), (50, -80)]:
        move_and_report(150, j2, j3, label="<-- -X reach")
    go_safe()

    print("\n=== Max +Z: vertical reach ===")
    go_safe()
    for j2, j3 in [(10, -10), (20, -20), (10, -30), (5, -5)]:
        move_and_report(0, j2, j3, label="<-- +Z reach")
    go_safe()

    print("\n=== Exploration complete! ===")

    if reached:
        xs = [r["actual"][0] for r in reached]
        ys = [r["actual"][1] for r in reached]
        zs = [r["actual"][2] for r in reached]

        print("\n" + "=" * 50)
        print("  WORKSPACE SUMMARY (actual arm positions)")
        print("=" * 50)
        print("  X range: {:.0f} to {:.0f} mm".format(min(xs), max(xs)))
        print("  Y range: {:.0f} to {:.0f} mm".format(min(ys), max(ys)))
        print("  Z range: {:.0f} to {:.0f} mm".format(min(zs), max(zs)))
        print("")
        print("  Positions reached: {}".format(len(reached)))
        print("  Positions skipped: {} (safety limits)".format(len(skipped)))

        max_x_pos = max(reached, key=lambda r: r["actual"][0])
        min_x_pos = min(reached, key=lambda r: r["actual"][0])
        max_y_pos = max(reached, key=lambda r: r["actual"][1])
        max_z_pos = max(reached, key=lambda r: r["actual"][2])

        print("")
        print("  Max +X: ({:.0f},{:.0f},{:.0f}) at J1={:.0f} J2={:.0f} J3={:.0f}".format(
            *max_x_pos["actual"], *max_x_pos["joints"]))
        print("  Max -X: ({:.0f},{:.0f},{:.0f}) at J1={:.0f} J2={:.0f} J3={:.0f}".format(
            *min_x_pos["actual"], *min_x_pos["joints"]))
        print("  Max +Y: ({:.0f},{:.0f},{:.0f}) at J1={:.0f} J2={:.0f} J3={:.0f}".format(
            *max_y_pos["actual"], *max_y_pos["joints"]))
        print("  Max +Z: ({:.0f},{:.0f},{:.0f}) at J1={:.0f} J2={:.0f} J3={:.0f}".format(
            *max_z_pos["actual"], *max_z_pos["joints"]))

        horiz = [math.sqrt(r["actual"][0]**2 + r["actual"][1]**2) for r in reached]
        max_h_idx = horiz.index(max(horiz))
        max_h = reached[max_h_idx]
        print("  Max horizontal reach: {:.0f}mm at ({:.0f},{:.0f},{:.0f})".format(
            max(horiz), *max_h["actual"]))

        print("")
        print("  Recommended coord_server WORKSPACE limits:")
        print("    WORKSPACE_MIN = [{:.0f}, 0, {:.0f}]".format(min(xs), max(80, min(zs))))
        print("    WORKSPACE_MAX = [{:.0f}, {:.0f}, {:.0f}]".format(max(xs), max(ys), max(zs)))
        print("=" * 50)

except KeyboardInterrupt:
    print("\n\nInterrupted!")
finally:
    print("\n--- Returning to zero ---")
    go_home(duration=3.0)
    print("Disabling arm...")
    piper.DisablePiper()
    print("Done.")
