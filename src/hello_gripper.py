#!/usr/bin/env python3
"""
Hello World — Gripper only.

Opens and closes the gripper a few times.
Requires CAN interface to be active:
  sudo ip link set can0 up type can bitrate 1000000
"""
import time
from piper_sdk import C_PiperInterface_V2

piper = C_PiperInterface_V2("can0")
piper.ConnectPort()

print("Firmware:", piper.GetPiperFirmwareVersion())

print("Enabling arm...")
while not piper.EnablePiper():
    time.sleep(0.01)
print("Enabled!")

# Initialize gripper: clear errors then enable (no sleeps — matches official demo)
piper.GripperCtrl(0, 1000, 0x02, 0)
piper.GripperCtrl(0, 1000, 0x01, 0)

# Warm up: send gripper commands in a tight loop to ensure firmware accepts them
print("Warming up gripper...")
for _ in range(200):
    piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
    piper.GripperCtrl(0, 1000, 0x01, 0)
    time.sleep(0.005)
print("Gripper initialized.\n")

EFFORT = 1000  # 1 N/m

def set_gripper(opening_mm, duration=2.0):
    """Send gripper to a position and hold."""
    units = int(opening_mm * 1000)  # mm → 0.001mm
    steps = int(duration / 0.005)
    for _ in range(steps):
        piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
        piper.GripperCtrl(units, EFFORT, 0x01, 0)
        gp = piper.GetArmGripperMsgs().gripper_state
        actual = gp.grippers_angle / 1000.0
        print(f"  Target: {opening_mm:.0f}mm  Actual: {actual:.1f}mm", end="\r")
        time.sleep(0.005)
    print()

print("--- Sweeping gripper range: 0mm → 80mm in 5mm steps ---")
for mm in range(0, 85, 5):
    set_gripper(float(mm), duration=1.0)

print("\n--- Closing ---")
set_gripper(0.0)

print("\nDisabling arm...")
piper.DisablePiper()
print("Done.")
