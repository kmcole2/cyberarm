#!/usr/bin/env python3
"""
Hello World — Piper SDK on Linux

Connects to the arm, enables it, and moves it up and down.
Requires CAN interface to be active:
  bash can_activate.sh can0 1000000
"""
import time
from piper_sdk import C_PiperInterface_V2

piper = C_PiperInterface_V2("can0")
piper.ConnectPort()
time.sleep(0.025)

print("Firmware:", piper.GetPiperFirmwareVersion())

print("Enabling arm...")
while not piper.EnablePiper():
    time.sleep(0.01)
print("Enabled!")

# Joint angles in 0.001 degrees (millidegrees)
# Joint 2 is the shoulder — controls up/down motion
# Joint 2 range: [0°, 180°]
POS_UP = [0, 20_000, -40_000, 0, 0, 0]    # shoulder at 20°, elbow at -40°
POS_DOWN = [0, 70_000, -100_000, 0, 0, 0]  # shoulder at 70°, elbow at -100°

positions = [POS_UP, POS_DOWN]
labels = ["UP", "DOWN"]

print("Moving arm up and down (Ctrl+C to stop)...\n")
try:
    cycle = 0
    while cycle < 5:
        idx = cycle % 2
        target = positions[idx]
        print(f"--- Moving {labels[idx]} ---")

        for _ in range(300):
            piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
            piper.JointCtrl(*target)

            js = piper.GetArmJointMsgs().joint_state
            print(
                f"  Target: J2={target[1]/1000:.0f}° J3={target[2]/1000:.0f}°  |  "
                f"Actual: J2={js.joint_2/1000:.1f}° J3={js.joint_3/1000:.1f}°",
                end="\r",
            )
            time.sleep(0.005)

        print()
        time.sleep(0.5)
        cycle += 1

except KeyboardInterrupt:
    print("\n\nStopping...")

print("Returning to zero...")
for _ in range(300):
    piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    time.sleep(0.005)

print("Disabling arm...")
piper.DisablePiper()
print("Done.")
