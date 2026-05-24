#!/usr/bin/env python3
"""
Explore the Piper arm's reachable workspace by sampling FK across joint configurations.
Prints the min/max for each axis (X, Y, Z) and the max horizontal reach.
"""
import math
import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from piper_ik import PiperIK, JOINT_LIMITS_RAD

ik = PiperIK()

SAMPLES_PER_JOINT = 12

j1_range = np.linspace(JOINT_LIMITS_RAD[0][0], JOINT_LIMITS_RAD[0][1], SAMPLES_PER_JOINT)
j2_range = np.linspace(JOINT_LIMITS_RAD[1][0], JOINT_LIMITS_RAD[1][1], SAMPLES_PER_JOINT)
j3_range = np.linspace(JOINT_LIMITS_RAD[2][0], JOINT_LIMITS_RAD[2][1], SAMPLES_PER_JOINT)

# J4/J5/J6 mostly affect orientation, not position much — sample a few
j4_range = np.linspace(JOINT_LIMITS_RAD[3][0], JOINT_LIMITS_RAD[3][1], 5)
j5_range = np.linspace(JOINT_LIMITS_RAD[4][0], JOINT_LIMITS_RAD[4][1], 5)

x_min, x_max = float('inf'), float('-inf')
y_min, y_max = float('inf'), float('-inf')
z_min, z_max = float('inf'), float('-inf')
max_horiz = 0.0
max_reach_config = None

total = len(j1_range) * len(j2_range) * len(j3_range) * len(j4_range) * len(j5_range)
print("Sampling {} configurations (J1 x J2 x J3 x J4 x J5)...".format(total))

count = 0
for j1 in j1_range:
    for j2 in j2_range:
        for j3 in j3_range:
            for j4 in j4_range:
                for j5 in j5_range:
                    joints = [j1, j2, j3, j4, j5, 0.0]
                    result = ik.forward(joints)
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
                        max_reach_config = joints[:]

                    count += 1
                    if count % 10000 == 0:
                        print("  {}/{} sampled...".format(count, total))

print("\n=== Workspace Bounds (mm) ===")
print("  X: {:.1f} to {:.1f}".format(x_min, x_max))
print("  Y: {:.1f} to {:.1f}".format(y_min, y_max))
print("  Z: {:.1f} to {:.1f}".format(z_min, z_max))
print("  Max horizontal reach: {:.1f}mm".format(max_horiz))

if max_reach_config:
    degs = [math.degrees(j) for j in max_reach_config]
    pos = ik.forward(max_reach_config)
    print("\nMax reach config (degrees): [{:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}]".format(*degs))
    print("  -> pos=({:.1f}, {:.1f}, {:.1f})".format(pos[0], pos[1], pos[2]))

# Test specific Y values with IK to find practical max
print("\n=== IK Reachability Test Along Y Axis ===")
print("Testing which Y values the IK can actually reach (x=0, z=200):")
ik2 = PiperIK()
for y_test in range(0, 700, 25):
    joints, ok = ik2.solve_position_only([0.0, float(y_test), 200.0])
    if ok:
        actual = ik2.forward(joints)
        print("  y={:4d}mm -> OK  (actual: x={:.1f} y={:.1f} z={:.1f})".format(
            y_test, actual[0], actual[1], actual[2]))
    else:
        print("  y={:4d}mm -> FAIL (out of reach)".format(y_test))
