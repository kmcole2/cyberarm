#!/usr/bin/env python3
"""
Hello World — Piper SDK on Linux

Connects to the arm, enables it, moves up/down, rotates the wrist,
and opens/closes the gripper.

Requires CAN interface to be active:
  sudo ip link set can0 up type can bitrate 1000000
"""
import time
import math
from piper_sdk import C_PiperInterface_V2

piper = C_PiperInterface_V2("can0")
piper.ConnectPort()
time.sleep(0.025)

print("Firmware:", piper.GetPiperFirmwareVersion())

print("Enabling arm...")
while not piper.EnablePiper():
    time.sleep(0.01)
print("Enabled!")

# Joint angles in millidegrees (0.001°)
# J1: base rotation, J2: shoulder, J3: elbow
# J4: forearm roll, J5: wrist pitch, J6: wrist yaw

SPEED = 50

def send(j1, j2, j3, j4, j5, j6, gripper_mm=0.0, duration=1.5):
    """Send joint target and gripper for a duration."""
    target = [j1, j2, j3, j4, j5, j6]
    gripper_units = int(gripper_mm * 1000)
    steps = int(duration / 0.005)
    for i in range(steps):
        piper.MotionCtrl_2(0x01, 0x01, SPEED, 0x00)
        piper.JointCtrl(*target)
        piper.GripperCtrl(gripper_units, 1000, 0x01, 0)

        js = piper.GetArmJointMsgs().joint_state
        print(
            f"  J1={js.joint_1/1000:.0f}° J2={js.joint_2/1000:.0f}° "
            f"J3={js.joint_3/1000:.0f}° J6={js.joint_6/1000:.0f}° "
            f"grip={gripper_mm:.0f}mm",
            end="\r",
        )
        time.sleep(0.005)
    print()


print("\n--- Phase 1: Raise arm, open gripper ---")
send(0, 40_000, -60_000, 0, 0, 0, gripper_mm=50.0)

print("--- Phase 2: Rotate J6 through full range (±120°) ---")
send(0, 40_000, -60_000, 0, 0, 120_000, gripper_mm=50.0)
send(0, 40_000, -60_000, 0, 0, -120_000, gripper_mm=50.0)
send(0, 40_000, -60_000, 0, 0, 0, gripper_mm=50.0)

print("--- Phase 3: Rotate J4 forearm roll (±100°) ---")
send(0, 40_000, -60_000, 100_000, 0, 0, gripper_mm=30.0)
send(0, 40_000, -60_000, -100_000, 0, 0, gripper_mm=30.0)
send(0, 40_000, -60_000, 0, 0, 0, gripper_mm=30.0)

print("--- Phase 4: Wrist pitch J5 (±70°) + close gripper ---")
send(0, 40_000, -60_000, 0, 70_000, 0, gripper_mm=0.0)
send(0, 40_000, -60_000, 0, -70_000, 0, gripper_mm=0.0)
send(0, 40_000, -60_000, 0, 0, 0, gripper_mm=0.0)

print("--- Phase 5: Combined motion — sweep base + rotate + gripper cycle ---")
for cycle in range(2):
    send(60_000, 50_000, -80_000, 50_000, 30_000, 60_000, gripper_mm=50.0, duration=2.0)
    send(-60_000, 30_000, -50_000, -50_000, -30_000, -60_000, gripper_mm=0.0, duration=2.0)

print("\n--- Returning to zero ---")
send(0, 0, 0, 0, 0, 0, gripper_mm=0.0, duration=2.0)

print("Disabling arm...")
piper.DisablePiper()
print("Done.")
