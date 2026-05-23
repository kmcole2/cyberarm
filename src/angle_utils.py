"""Angle calculation utilities for mapping MediaPipe landmarks to Piper joint angles."""

import math
import numpy as np


JOINT_LIMITS_DEG = {
    "j1": (-150.0, 150.0),
    "j2": (0.0, 180.0),
    "j3": (-170.0, 0.0),
    "j4": (-100.0, 100.0),
    "j5": (-70.0, 70.0),
    "j6": (-120.0, 120.0),
}

SAFETY_MARGIN = 0.7
SAFE_LIMITS = {
    k: (v[0] * SAFETY_MARGIN, v[1] * SAFETY_MARGIN)
    for k, v in JOINT_LIMITS_DEG.items()
}


def angle_between_vectors(v1, v2):
    """Return angle in degrees between two 3D vectors."""
    v1_n = v1 / (np.linalg.norm(v1) + 1e-8)
    v2_n = v2 / (np.linalg.norm(v2) + 1e-8)
    dot = np.clip(np.dot(v1_n, v2_n), -1.0, 1.0)
    return math.degrees(math.acos(dot))


def landmarks_to_piper_joints(world_landmarks, side="right"):
    """
    Convert MediaPipe pose_world_landmarks to Piper joint angles in degrees.

    Args:
        world_landmarks: mediapipe pose_world_landmarks.landmark list
        side: 'right' or 'left' arm to track

    Returns:
        List of 6 joint angles in degrees [j1, j2, j3, j4, j5, j6]
    """
    lm = world_landmarks

    if side == "right":
        shoulder = np.array([lm[12].x, lm[12].y, lm[12].z])
        elbow = np.array([lm[14].x, lm[14].y, lm[14].z])
        wrist = np.array([lm[16].x, lm[16].y, lm[16].z])
        hip = np.array([lm[24].x, lm[24].y, lm[24].z])
    else:
        shoulder = np.array([lm[11].x, lm[11].y, lm[11].z])
        elbow = np.array([lm[13].x, lm[13].y, lm[13].z])
        wrist = np.array([lm[15].x, lm[15].y, lm[15].z])
        hip = np.array([lm[23].x, lm[23].y, lm[23].z])

    # J1: Base rotation — horizontal angle of the upper arm
    # Project shoulder→elbow onto XZ plane (horizontal), measure from forward (-Z)
    arm_vec = elbow - shoulder
    j1 = math.degrees(math.atan2(arm_vec[0], -arm_vec[2]))

    # J2: Shoulder pitch — angle between torso vertical and upper arm
    # MediaPipe Y axis points down, so hip-shoulder is the "up" direction of torso
    torso_up = shoulder - hip
    upper_arm = elbow - shoulder
    j2 = angle_between_vectors(torso_up, upper_arm)

    # J3: Elbow bend — angle at elbow joint
    # 180° = straight arm, smaller = bent
    # Piper convention: 0° = straight, negative = bent
    forearm = wrist - elbow
    upper_arm_rev = shoulder - elbow
    elbow_angle = angle_between_vectors(upper_arm_rev, forearm)
    j3 = -(180.0 - elbow_angle)

    # J4-J6: Wrist degrees of freedom (Phase 2 — zeros for now)
    j4 = 0.0
    j5 = 0.0
    j6 = 0.0

    return [j1, j2, j3, j4, j5, j6]


def check_visibility(world_landmarks, side="right", threshold=0.6):
    """Return True if all required landmarks are visible above threshold."""
    if side == "right":
        indices = [12, 14, 16, 24]
    else:
        indices = [11, 13, 15, 23]

    return all(world_landmarks[i].visibility > threshold for i in indices)


class AngleSmoother:
    """Exponential moving average filter for joint angles."""

    def __init__(self, alpha=0.3, num_joints=6):
        self.alpha = alpha
        self.prev = [0.0] * num_joints
        self.initialized = False

    def smooth(self, raw_angles):
        if not self.initialized:
            self.prev = list(raw_angles)
            self.initialized = True
            return list(raw_angles)

        smoothed = []
        for i, raw in enumerate(raw_angles):
            s = self.alpha * raw + (1 - self.alpha) * self.prev[i]
            smoothed.append(s)
        self.prev = smoothed
        return smoothed


class VelocityLimiter:
    """Clamp per-frame angle changes to prevent jerky movements."""

    def __init__(self, max_step_deg=5.0):
        self.max_step = max_step_deg
        self.prev = None

    def limit(self, angles):
        if self.prev is None:
            self.prev = list(angles)
            return list(angles)

        limited = []
        for i, a in enumerate(angles):
            delta = a - self.prev[i]
            clamped_delta = max(-self.max_step, min(self.max_step, delta))
            limited.append(self.prev[i] + clamped_delta)
        self.prev = limited
        return limited


def clamp_to_limits(angles, use_safety_margin=True):
    """Clamp joint angles to their allowed ranges."""
    limits = SAFE_LIMITS if use_safety_margin else JOINT_LIMITS_DEG
    keys = ["j1", "j2", "j3", "j4", "j5", "j6"]
    clamped = []
    for i, a in enumerate(angles):
        lo, hi = limits[keys[i]]
        clamped.append(max(lo, min(hi, a)))
    return clamped


def degrees_to_millidegrees(angles):
    """Convert a list of angles from degrees to millidegrees (int)."""
    return [int(a * 1000) for a in angles]
