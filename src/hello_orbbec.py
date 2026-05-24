#!/usr/bin/env python3

"""
Hello World for Orbbec depth camera.

Opens a live viewer showing the color and depth streams side by side.
Press Q or ESC to quit the viewer.
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sdks", "pyorbbecsdk"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sdks", "pyorbbecsdk", "examples"))

import cv2
import numpy as np
from pyorbbecsdk import Pipeline, OBError, OBFormat
from utils import frame_to_bgr_image

MIN_DEPTH = 20
MAX_DEPTH = 5000

try:
    pipeline = Pipeline()
    pipeline.start()
except OBError as e:
    print("Error: {}".format(e))
    print("Please connect an Orbbec camera and try again.")
    sys.exit(1)

print("Pipeline started. Press Q or ESC to quit.")

cv2.namedWindow("Orbbec Viewer", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Orbbec Viewer", 1280, 480)

try:
    while True:
        frames = pipeline.wait_for_frames(1000)
        if frames is None:
            continue

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        panels = []

        # Color
        if color_frame is not None:
            bgr = frame_to_bgr_image(color_frame)
            if bgr is not None:
                panels.append(bgr)

        # Depth
        if depth_frame is not None:
            w = depth_frame.get_width()
            h = depth_frame.get_height()
            scale = depth_frame.get_depth_scale()
            raw = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape((h, w))
            depth_mm = raw.astype(np.float32) * scale

            depth_norm = np.clip((depth_mm - MIN_DEPTH) / (MAX_DEPTH - MIN_DEPTH), 0, 1)
            depth_colored = cv2.applyColorMap((depth_norm * 255).astype(np.uint8), cv2.COLORMAP_JET)

            center_mm = depth_mm[h // 2, w // 2]
            cv2.putText(depth_colored, "Center: {:.0f} mm".format(center_mm),
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            panels.append(depth_colored)

        if panels:
            if len(panels) == 2:
                h = min(panels[0].shape[0], panels[1].shape[0])
                panels = [cv2.resize(p, (640, h)) for p in panels]
                combined = np.hstack(panels)
            else:
                combined = panels[0]
            cv2.imshow("Orbbec Viewer", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break

except KeyboardInterrupt:
    print("\nInterrupted.")

pipeline.stop()
cv2.destroyAllWindows()
print("Done.")
