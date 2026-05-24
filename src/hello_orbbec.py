#!/usr/bin/env python3

"""
Hello World for Orbbec depth camera.

Prints device info, then opens a live viewer showing the color and depth streams.
Press Q or ESC to quit the viewer.
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sdks", "pyorbbecsdk", "examples"))

import cv2
import numpy as np
from pyorbbecsdk import (
    Config,
    Context,
    OBLogLevel,
    OBSensorType,
    OBError,
    Pipeline,
)
from utils import frame_to_bgr_image

Context.set_logger_to_console(OBLogLevel.WARNING)

# --- Device discovery ---
ctx = Context()
device_list = ctx.query_devices()
if device_list.get_count() == 0:
    print("No Orbbec device found - check the USB cable and try another port.")
    raise SystemExit(1)

device = device_list.get_device_by_index(0)
info = device.get_device_info()
print("=== Orbbec Device ===")
print("  Name:       {}".format(info.get_name()))
print("  Serial:     {}".format(info.get_serial_number()))
print("  Firmware:   {}".format(info.get_firmware_version()))
print("  Connection: {}".format(info.get_connection_type()))

# --- Show available stream profiles ---
pipeline = Pipeline(device)
has_color = False
has_depth = False

for sensor_type, label in [
    (OBSensorType.DEPTH_SENSOR, "Depth"),
    (OBSensorType.COLOR_SENSOR, "Color"),
]:
    try:
        profiles = pipeline.get_stream_profile_list(sensor_type)
        p = profiles.get_default_video_stream_profile()
        print("  {}: {}x{} @ {} fps".format(label, p.get_width(), p.get_height(), p.get_fps()))
        if sensor_type == OBSensorType.COLOR_SENSOR:
            has_color = True
        else:
            has_depth = True
    except OBError:
        print("  {}: not available on this device".format(label))

# --- Configure streams ---
config = Config()
if has_color:
    color_profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    config.enable_stream(color_profiles.get_default_video_stream_profile())
if has_depth:
    depth_profiles = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    config.enable_stream(depth_profiles.get_default_video_stream_profile())

# --- Live viewer ---
print("\nStarting live viewer (press Q or ESC to quit)...")
pipeline.start(config)

try:
    while True:
        frameset = pipeline.wait_for_frames(100)
        if frameset is None:
            continue

        # Color stream
        if has_color:
            color_frame = frameset.get_color_frame()
            if color_frame:
                bgr = frame_to_bgr_image(color_frame)
                if bgr is not None:
                    cv2.imshow("Orbbec - Color", bgr)

        # Depth stream (colorized for visualization)
        if has_depth:
            depth_frame = frameset.get_depth_frame()
            if depth_frame:
                w = depth_frame.get_width()
                h = depth_frame.get_height()
                scale = depth_frame.get_depth_scale()
                raw = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape((h, w))
                depth_mm = raw.astype(np.float32) * scale
                # Normalize to 0-255 for display (clip at 5 meters)
                depth_display = np.clip(depth_mm / 5000.0, 0, 1)
                depth_colormap = cv2.applyColorMap(
                    (depth_display * 255).astype(np.uint8), cv2.COLORMAP_JET
                )
                # Show center distance
                center_mm = depth_mm[h // 2, w // 2]
                cv2.putText(
                    depth_colormap,
                    "Center: {:.0f} mm".format(center_mm),
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                )
                cv2.imshow("Orbbec - Depth", depth_colormap)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break

except KeyboardInterrupt:
    print("\nInterrupted.")

pipeline.stop()
cv2.destroyAllWindows()
print("Done! Your Orbbec camera is working.")
