#!/usr/bin/env python3
"""
Query the Piper arm's hardware limits and derive the XYZ workspace range.

Reads joint angle limits from the firmware, then samples FK to compute
the reachable workspace bounds.
"""
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from can_setup import ensure_can
from piper_ik import PiperIK

ensure_can()

from piper_sdk import C_PiperInterface_V2

piper = C_PiperInterface_V2("can0")
piper.ConnectPort()
time.sleep(1.0)

print("Firmware:", piper.GetPiperFirmwareVersion())

# --- Query joint limits from hardware ---
limits_msg = piper.GetAllMotorAngleLimitMaxSpd()
lim = limits_msg.all_motor_angle_limit_max_spd

motors = [lim.motor_1, lim.motor_2, lim.motor_3, lim.motor_4, lim.motor_5, lim.motor_6]

print("\n=== Hardware Joint Limits ===")
joint_limits_rad = []
for i, m in enumerate(motors):
    min_deg = m.min_angle_limit / 10.0
    max_deg = m.max_angle_limit / 10.0
    max_spd = m.max_joint_spd / 1000.0
    print("  J{}: {:.1f} to {:.1f} deg  (max speed: {:.2f} rad/s)".format(
        i + 1, min_deg, max_deg, max_spd))
    joint_limits_rad.append((math.radians(min_deg), math.radians(max_deg)))

# --- Query gripper range ---
piper.ArmParamEnquiryAndConfig(param_enquiry=0x04)
time.sleep(0.5)
gripper_msg = piper.GetGripperTeachingPendantParamFeedback()
gp = gripper_msg.arm_gripper_teaching_param_feedback
print("\n=== Gripper Parameters ===")
print("  Max range config: {}mm".format(gp.max_range_config))
print("  Teaching range percent: {}".format(gp.teaching_range_per))

# --- Query end effector velocity/acceleration ---
piper.ArmParamEnquiryAndConfig(param_enquiry=0x01)
time.sleep(0.5)
vel_acc = piper.GetCurrentEndVelAndAccParam()
va = vel_acc.current_end_vel_acc_param
print("\n=== End Effector Limits ===")
print("  Max linear velocity: {:.3f} m/s".format(va.end_max_linear_vel / 1000.0))
print("  Max angular velocity: {:.3f} rad/s".format(va.end_max_angular_vel / 1000.0))
print("  Max linear acceleration: {:.3f} m/s^2".format(va.end_max_linear_acc / 1000.0))
print("  Max angular acceleration: {:.3f} rad/s^2".format(va.end_max_angular_acc / 1000.0))

# --- Query max acceleration per motor ---
acc_msg = piper.GetAllMotorMaxAccLimit()
acc = acc_msg.all_motor_max_acc_limit
acc_motors = [acc.motor_1, acc.motor_2, acc.motor_3, acc.motor_4, acc.motor_5, acc.motor_6]
print("\n=== Motor Max Acceleration ===")
for i, m in enumerate(acc_motors):
    print("  J{}: {:.3f} rad/s^2".format(i + 1, m.max_joint_acc / 1000.0))

# --- Derive XYZ workspace via FK sampling ---
print("\n=== Computing Workspace (FK sampling) ===")

ik = PiperIK()
SAMPLES = 15

ranges = []
for lo, hi in joint_limits_rad[:3]:
    ranges.append(np.linspace(lo, hi, SAMPLES))

j4_range = np.linspace(joint_limits_rad[3][0], joint_limits_rad[3][1], 5)
j5_range = np.linspace(joint_limits_rad[4][0], joint_limits_rad[4][1], 5)

total = SAMPLES ** 3 * len(j4_range) * len(j5_range)
print("  Sampling {} configurations...".format(total))

x_min, x_max = float('inf'), float('-inf')
y_min, y_max = float('inf'), float('-inf')
z_min, z_max = float('inf'), float('-inf')
max_horiz = 0.0

# Also track safe bounds (Y >= 0, Z >= 80)
sx_min, sx_max = float('inf'), float('-inf')
sy_max = float('-inf')
sz_min, sz_max = float('inf'), float('-inf')

count = 0
for j1 in ranges[0]:
    for j2 in ranges[1]:
        for j3 in ranges[2]:
            for j4 in j4_range:
                for j5 in j5_range:
                    result = ik.forward([j1, j2, j3, j4, j5, 0.0])
                    x, y, z = result[0], result[1], result[2]

                    x_min = min(x_min, x)
                    x_max = max(x_max, x)
                    y_min = min(y_min, y)
                    y_max = max(y_max, y)
                    z_min = min(z_min, z)
                    z_max = max(z_max, z)
                    horiz = math.sqrt(x * x + y * y)
                    if horiz > max_horiz:
                        max_horiz = horiz

                    if y >= 0 and z >= 80:
                        sx_min = min(sx_min, x)
                        sx_max = max(sx_max, x)
                        sy_max = max(sy_max, y)
                        sz_min = min(sz_min, z)
                        sz_max = max(sz_max, z)

                    count += 1
                    if count % 20000 == 0:
                        print("  {}/{} sampled...".format(count, total))

print("\n" + "=" * 55)
print("  FULL WORKSPACE (no safety constraints)")
print("=" * 55)
print("  X: {:.0f} to {:.0f} mm".format(x_min, x_max))
print("  Y: {:.0f} to {:.0f} mm".format(y_min, y_max))
print("  Z: {:.0f} to {:.0f} mm".format(z_min, z_max))
print("  Max horizontal reach: {:.0f} mm".format(max_horiz))

print("\n" + "=" * 55)
print("  SAFE WORKSPACE (Y >= 0, Z >= 80mm)")
print("=" * 55)
if sx_min != float('inf'):
    print("  X: {:.0f} to {:.0f} mm".format(sx_min, sx_max))
    print("  Y: 0 to {:.0f} mm".format(sy_max))
    print("  Z: {:.0f} to {:.0f} mm".format(sz_min, sz_max))
    print("")
    print("  Recommended coord_server limits:")
    print("    WORKSPACE_MIN = [{:.0f}, 0, {:.0f}]".format(sx_min, max(80, sz_min)))
    print("    WORKSPACE_MAX = [{:.0f}, {:.0f}, {:.0f}]".format(sx_max, sy_max, sz_max))
else:
    print("  No safe positions found!")

if gp.max_range_config > 0:
    print("\n  Gripper range: 0 to {}mm".format(gp.max_range_config))

print("=" * 55)

piper.DisablePiper()
print("\nDone.")
