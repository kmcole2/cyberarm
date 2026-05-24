#!/usr/bin/env python3
"""
Hello World — Gripper diagnostic + control.

Opens and closes the gripper while printing full diagnostic info
so we can figure out why it might not be responding.
"""
import time
from can_setup import ensure_can
from piper_sdk import C_PiperInterface_V2

ensure_can()

piper = C_PiperInterface_V2("can0")
piper.ConnectPort()

print("Firmware:", piper.GetPiperFirmwareVersion())

# Print arm status before enabling
status = piper.GetArmStatus().arm_status
print(f"Arm status before enable: status={status.arm_status} mode={status.ctrl_mode} err={status.err_status}")

print("\nEnabling arm...")
while not piper.EnablePiper():
    time.sleep(0.01)
print("Arm enabled!")

# Check motor enable states
enable_list = piper.GetArmEnableStatus()
print(f"Motor enable status (1-6): {enable_list}")

# Check arm status after enabling
status = piper.GetArmStatus().arm_status
print(f"Arm status after enable: status={status.arm_status} mode={status.ctrl_mode} err={status.err_status}")

# Read gripper state BEFORE init
gp = piper.GetArmGripperMsgs().gripper_state
print(f"\nGripper state before init: angle={gp.grippers_angle} effort={gp.grippers_effort} status_code={gp.status_code}")

# Initialize gripper: clear errors then enable
print("\nSending GripperCtrl(0, 1000, 0x02, 0) — disable + clear error...")
piper.GripperCtrl(0, 1000, 0x02, 0)
time.sleep(0.5)

gp = piper.GetArmGripperMsgs().gripper_state
print(f"Gripper state after 0x02: angle={gp.grippers_angle} effort={gp.grippers_effort} status_code={gp.status_code}")

print("Sending GripperCtrl(0, 1000, 0x01, 0) — enable...")
piper.GripperCtrl(0, 1000, 0x01, 0)
time.sleep(0.5)

gp = piper.GetArmGripperMsgs().gripper_state
print(f"Gripper state after 0x01: angle={gp.grippers_angle} effort={gp.grippers_effort} status_code={gp.status_code}")

# Also try 0x03 (enable + clear error)
print("Sending GripperCtrl(0, 1000, 0x03, 0) — enable + clear error...")
piper.GripperCtrl(0, 1000, 0x03, 0)
time.sleep(0.5)

gp = piper.GetArmGripperMsgs().gripper_state
print(f"Gripper state after 0x03: angle={gp.grippers_angle} effort={gp.grippers_effort} status_code={gp.status_code}")

# Warm up with rapid commands
print("\nWarming up (sending 500 commands at 5ms interval)...")
for i in range(500):
    piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
    piper.GripperCtrl(0, 1000, 0x01, 0)
    if i % 100 == 99:
        gp = piper.GetArmGripperMsgs().gripper_state
        print(f"  [{i+1}/500] angle={gp.grippers_angle} effort={gp.grippers_effort} status_code={gp.status_code}")
    time.sleep(0.005)

print("\nGripper initialized. Starting movement test...\n")

EFFORT = 1000

def set_gripper(opening_mm, duration=2.0):
    """Send gripper to a position and hold."""
    units = int(opening_mm * 1000)  # mm → 0.001mm
    steps = int(duration / 0.005)
    for step in range(steps):
        piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
        piper.GripperCtrl(units, EFFORT, 0x01, 0)
        gp = piper.GetArmGripperMsgs().gripper_state
        actual = gp.grippers_angle / 1000.0
        if step % 50 == 0:
            print(f"  Target: {opening_mm:.0f}mm  Actual: {actual:.1f}mm  effort={gp.grippers_effort}  status={gp.status_code}")
    print()

print("--- Test: Open to 50mm ---")
set_gripper(50.0, duration=3.0)

print("--- Test: Close to 0mm ---")
set_gripper(0.0, duration=3.0)

print("--- Test: Open to 70mm ---")
set_gripper(70.0, duration=3.0)

print("--- Test: Close to 0mm ---")
set_gripper(0.0, duration=3.0)

print("\nDisabling arm...")
piper.DisablePiper()
print("Done.")
