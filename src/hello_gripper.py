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
time.sleep(0.5)

print("Firmware:", piper.GetPiperFirmwareVersion())

status = piper.GetArmStatus().arm_status
print("Arm status before enable: status={} mode={} err={}".format(
    status.arm_status, status.ctrl_mode, status.err_status))

print("\nEnabling arm...")
while not piper.EnablePiper():
    time.sleep(0.01)
print("Arm enabled!")
time.sleep(0.5)

enable_list = piper.GetArmEnableStatus()
print("Motor enable status (1-6): {}".format(enable_list))

status = piper.GetArmStatus().arm_status
print("Arm status after enable: status={} mode={} err={}".format(
    status.arm_status, status.ctrl_mode, status.err_status))

gp = piper.GetArmGripperMsgs().gripper_state
print("\nGripper BEFORE init: angle={} effort={} status_code={}".format(
    gp.grippers_angle, gp.grippers_effort, gp.status_code))

# Initialize gripper with pauses to avoid CAN bus overload
print("\nStep 1: GripperCtrl(0, 1000, 0x02, 0) — disable + clear error")
piper.GripperCtrl(0, 1000, 0x02, 0)
time.sleep(1.0)

gp = piper.GetArmGripperMsgs().gripper_state
print("  Gripper after 0x02: angle={} effort={} status_code={}".format(
    gp.grippers_angle, gp.grippers_effort, gp.status_code))

print("Step 2: GripperCtrl(0, 1000, 0x01, 0) — enable")
piper.GripperCtrl(0, 1000, 0x01, 0)
time.sleep(1.0)

gp = piper.GetArmGripperMsgs().gripper_state
print("  Gripper after 0x01: angle={} effort={} status_code={}".format(
    gp.grippers_angle, gp.grippers_effort, gp.status_code))

# Slow warm-up: 10ms interval instead of 5ms
print("\nStep 3: Warm up (200 commands at 10ms interval)...")
for i in range(200):
    piper.GripperCtrl(0, 1000, 0x01, 0)
    if i % 50 == 49:
        gp = piper.GetArmGripperMsgs().gripper_state
        print("  [{}/200] angle={} effort={} status_code={}".format(
            i + 1, gp.grippers_angle, gp.grippers_effort, gp.status_code))
    time.sleep(0.01)

print("\n--- Movement test ---\n")

EFFORT = 1000

def set_gripper(opening_mm, duration=3.0):
    units = int(opening_mm * 1000)
    steps = int(duration / 0.01)
    for step in range(steps):
        piper.GripperCtrl(units, EFFORT, 0x01, 0)
        if step % 30 == 0:
            gp = piper.GetArmGripperMsgs().gripper_state
            actual = gp.grippers_angle / 1000.0
            print("  Target: {}mm  Actual: {:.1f}mm  effort={}  status={}".format(
                opening_mm, actual, gp.grippers_effort, gp.status_code))
        time.sleep(0.01)
    print()

print("Opening to 50mm...")
set_gripper(50.0)

print("Closing to 0mm...")
set_gripper(0.0)

print("Opening to 70mm...")
set_gripper(70.0)

print("Closing to 0mm...")
set_gripper(0.0)

print("\nDisabling arm...")
piper.DisablePiper()
print("Done.")
