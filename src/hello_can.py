#!/usr/bin/env python3
"""
Hello World — python-can on can0

Sends a CAN message and listens for responses.
"""
import can
import time

CHANNEL = "can0"
BUSTYPE = "socketcan"
BITRATE = 1_000_000

bus = can.interface.Bus(channel=CHANNEL, bustype=BUSTYPE, bitrate=BITRATE)

# Send a message
msg = can.Message(
    arbitration_id=0x100,
    data=[0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x21, 0x00, 0x00],  # "Hello!"
    is_extended_id=False,
)
bus.send(msg)
print(f"Sent: id=0x{msg.arbitration_id:03X} data={msg.data.hex()}")

# Listen for replies
print("Listening for CAN messages (5 seconds)...")
deadline = time.time() + 5
while time.time() < deadline:
    rx = bus.recv(timeout=1.0)
    if rx:
        print(f"Received: id=0x{rx.arbitration_id:03X} data={rx.data.hex()}")

bus.shutdown()
print("Done.")
