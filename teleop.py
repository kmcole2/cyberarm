#!/usr/bin/env python3
"""
Piper Arm Teleoperation — macOS Setup & Control

Two modes:
  1. Hardware linkage: Both arms on same CAN bus, firmware handles teleop
  2. Software teleop: Read master joints via SDK, command slave via SDK

Usage:
  python3 teleop.py --mode setup_master   # Configure arm as master (teaching input)
  python3 teleop.py --mode setup_slave    # Configure arm as slave (motion output)
  python3 teleop.py --mode teleop         # Software-based teleop (two CAN adapters)
  python3 teleop.py --mode read           # Just read and print joint angles
"""
import argparse
import time
import sys
from piper_sdk import C_PiperInterface_V2


def detect_bustype():
    """Auto-detect whether to use gs_usb or slcan on macOS."""
    try:
        from gs_usb.gs_usb import GsUsb
        devs = GsUsb.scan()
        if devs:
            print("[OK] gs_usb adapter detected")
            return "gs_usb", "can0"
    except Exception:
        pass

    import glob
    serial_ports = glob.glob("/dev/tty.usb*") + glob.glob("/dev/cu.usb*")
    if serial_ports:
        port = serial_ports[0]
        print(f"[OK] Serial CAN adapter detected at {port}")
        return "slcan", port

    print("[WARN] No CAN adapter auto-detected. Defaulting to gs_usb.")
    print("       Plug in your AgileX adapter and retry.")
    return "gs_usb", "can0"


def create_interface(bustype, channel):
    """Create a Piper interface with macOS-compatible settings."""
    piper = C_PiperInterface_V2(
        can_name=channel,
        judge_flag=False,
        bustype=bustype,
    )
    piper.ConnectPort()
    time.sleep(0.1)
    return piper


def setup_master(bustype, channel):
    """Set arm as teaching input (master) arm."""
    print("Configuring arm as MASTER (teaching input)...")
    piper = create_interface(bustype, channel)
    piper.MasterSlaveConfig(0xFA, 0, 0, 0)
    print("Done. This arm is now the master.")
    print("Power cycle: turn on SLAVE first, then MASTER.")


def setup_slave(bustype, channel):
    """Set arm as motion output (slave) arm."""
    print("Configuring arm as SLAVE (motion output)...")
    piper = create_interface(bustype, channel)
    piper.MasterSlaveConfig(0xFC, 0, 0, 0)
    print("Done. This arm is now the slave.")
    print("Power cycle: turn on SLAVE first, then MASTER.")


def read_joints(bustype, channel):
    """Continuously read and print joint angles."""
    print("Reading joint angles (Ctrl+C to stop)...")
    piper = create_interface(bustype, channel)

    try:
        while True:
            joints = piper.GetArmJointMsgs()
            js = joints.joint_state
            print(
                f"J1:{js.joint_1/1000:7.2f}  "
                f"J2:{js.joint_2/1000:7.2f}  "
                f"J3:{js.joint_3/1000:7.2f}  "
                f"J4:{js.joint_4/1000:7.2f}  "
                f"J5:{js.joint_5/1000:7.2f}  "
                f"J6:{js.joint_6/1000:7.2f} deg  "
                f"({joints.Hz:.0f} Hz)",
                end="\r",
            )
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nStopped.")


def software_teleop(master_bustype, master_channel, slave_bustype, slave_channel):
    """
    Software-based teleop: read master arm joints, send to slave arm.
    Requires two separate CAN adapters.
    """
    print("Software teleop mode")
    print(f"  Master: {master_bustype} @ {master_channel}")
    print(f"  Slave:  {slave_bustype} @ {slave_channel}")
    print("Connecting...")

    master = create_interface(master_bustype, master_channel)
    slave = create_interface(slave_bustype, slave_channel)

    print("Enabling slave arm...")
    slave.MotionCtrl_2(0x01, 0x01, 100, 0xAD)
    while not slave.EnablePiper():
        time.sleep(0.01)
    slave.GripperCtrl(0, 1000, 0x01, 0)
    print("Slave enabled. Starting teleop (Ctrl+C to stop)...")

    try:
        while True:
            joints = master.GetArmJointMsgs()
            js = joints.joint_state

            slave.MotionCtrl_2(0x01, 0x01, 100, 0xAD)
            slave.JointCtrl(
                js.joint_1, js.joint_2, js.joint_3,
                js.joint_4, js.joint_5, js.joint_6,
            )

            gripper = master.GetArmGripperMsgs()
            if gripper:
                gs = gripper.gripper_state
                slave.GripperCtrl(abs(gs.grippers_angle), 1000, 0x01, 0)

            print(
                f"Teleop | J1:{js.joint_1/1000:6.1f} J2:{js.joint_2/1000:6.1f} "
                f"J3:{js.joint_3/1000:6.1f} J4:{js.joint_4/1000:6.1f} "
                f"J5:{js.joint_5/1000:6.1f} J6:{js.joint_6/1000:6.1f}",
                end="\r",
            )
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nDisabling slave arm...")
        slave.DisablePiper()
        print("Teleop stopped.")


def main():
    parser = argparse.ArgumentParser(description="Piper Arm Teleop (macOS)")
    parser.add_argument(
        "--mode",
        choices=["setup_master", "setup_slave", "teleop", "read"],
        default="read",
        help="Operation mode",
    )
    parser.add_argument("--bustype", help="CAN bus type (gs_usb, slcan, socketcan)")
    parser.add_argument("--channel", help="CAN channel name or serial port path")
    parser.add_argument("--slave-channel", help="Slave CAN channel (for teleop mode)")
    args = parser.parse_args()

    if args.bustype and args.channel:
        bustype, channel = args.bustype, args.channel
    else:
        bustype, channel = detect_bustype()

    if args.mode == "setup_master":
        setup_master(bustype, channel)
    elif args.mode == "setup_slave":
        setup_slave(bustype, channel)
    elif args.mode == "read":
        read_joints(bustype, channel)
    elif args.mode == "teleop":
        slave_channel = args.slave_channel or channel
        software_teleop(bustype, channel, bustype, slave_channel)


if __name__ == "__main__":
    main()
