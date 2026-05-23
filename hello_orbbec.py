#!/usr/bin/env python3
"""Hello World for Orbbec depth camera."""

import numpy as np
from pyorbbecsdk import (
    Context,
    OBLogLevel,
    OBSensorType,
    OBError,
    Pipeline,
)

Context.set_logger_to_console(OBLogLevel.WARNING)

# --- Device discovery ---
ctx = Context()
device_list = ctx.query_devices()
if device_list.get_count() == 0:
    print("No Orbbec device found - check the micro-USB cable and try another port.")
    raise SystemExit(1)

device = device_list.get_device_by_index(0)
info = device.get_device_info()
print("=== Orbbec Device ===")
print(f"  Name:       {info.get_name()}")
print(f"  Serial:     {info.get_serial_number()}")
print(f"  Firmware:   {info.get_firmware_version()}")
print(f"  Connection: {info.get_connection_type()}")

# --- Show available stream profiles ---
pipeline = Pipeline(device)
for sensor_type, label in [
    (OBSensorType.DEPTH_SENSOR, "Depth"),
    (OBSensorType.COLOR_SENSOR, "Color"),
]:
    try:
        profiles = pipeline.get_stream_profile_list(sensor_type)
        p = profiles.get_default_video_stream_profile()
        print(f"  {label}: {p.get_width()}x{p.get_height()} @ {p.get_fps()} fps")
    except OBError:
        print(f"  {label}: not available on this device")

# --- Capture one depth frame ---
print("\nCapturing a depth frame...")
pipeline.start()
frames = pipeline.wait_for_frames(3000)
if frames is None:
    print("Timed out waiting for frames.")
    pipeline.stop()
    raise SystemExit(1)

depth_frame = frames.get_depth_frame()
if depth_frame:
    w = depth_frame.get_width()
    h = depth_frame.get_height()
    scale = depth_frame.get_depth_scale()
    data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape((h, w))
    center_mm = data[h // 2, w // 2] * scale
    print(f"Depth frame: {w}x{h}")
    print(f"Center pixel distance: {center_mm:.0f} mm")
else:
    print("No depth data in this frameset (color-only device?).")

pipeline.stop()
print("\nDone! Your Orbbec camera is working.")
