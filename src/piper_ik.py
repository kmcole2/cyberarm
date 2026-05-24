#!/usr/bin/env python3
"""
Numerical Inverse Kinematics for the Piper arm.

Uses the SDK's forward kinematics as a black box and solves via
damped least squares (Levenberg-Marquardt style). This bypasses
EndPoseCtrl entirely — we compute joint angles and send them via JointCtrl.
"""

import math
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "piper_sdk"))
from piper_sdk.kinematics.piper_fk import C_PiperForwardKinematics


JOINT_LIMITS_RAD = [
    (-2.618, 2.618),   # J1: ±150°
    (0.0, 3.14),       # J2: 0 to 180°
    (-2.967, 0.0),     # J3: -170 to 0°
    (-1.745, 1.745),   # J4: ±100°
    (-1.22, 1.22),     # J5: ±70°
    (-2.0944, 2.0944), # J6: ±120°
]


class PiperIK:
    def __init__(self):
        self.fk = C_PiperForwardKinematics(dh_is_offset=0x01)
        self._q_prev = None

    def forward(self, joints_rad):
        """FK wrapper: joints (rad) -> [x, y, z, rx, ry, rz] (mm, degrees)."""
        result = self.fk.CalFK(joints_rad)
        return result[-1]

    def solve(self, target_xyz, target_rpy=None, max_iter=50, pos_tol=1.0, damping=0.5):
        """
        Solve IK for a target position (and optionally orientation).

        Args:
            target_xyz: [x, y, z] in mm
            target_rpy: [rx, ry, rz] in degrees (default: [0, 85, 0] = gripper forward)
            max_iter: iteration cap
            pos_tol: position convergence threshold in mm
            damping: Levenberg-Marquardt damping factor (higher = more stable, slower)

        Returns:
            (joints_rad, converged): list of 6 joint angles in radians, success bool
        """
        if target_rpy is None:
            target_rpy = [0.0, 85.0, 0.0]

        target = np.array(target_xyz + target_rpy, dtype=float)

        q = np.array(self._initial_guess(), dtype=float)

        for _ in range(max_iter):
            current = np.array(self.forward(q.tolist()))
            error = target - current

            pos_error = np.linalg.norm(error[:3])
            if pos_error < pos_tol:
                self._q_prev = q.copy()
                return q.tolist(), True

            J = self._jacobian(q)
            JT = J.T
            delta_q = JT @ np.linalg.solve(J @ JT + damping**2 * np.eye(6), error)

            q = q + delta_q
            self._clamp_joints(q)

        self._q_prev = q.copy()
        return q.tolist(), False

    def solve_position_only(self, target_xyz, target_rpy=None, max_iter=50, pos_tol=1.0, damping=0.5):
        """
        Solve IK prioritizing position over orientation.

        Uses a weighted error where position errors matter 10x more than
        orientation errors. Good for the coord_server use case where we
        care about XYZ and orientation is secondary.
        """
        if target_rpy is None:
            target_rpy = [0.0, 85.0, 0.0]

        target = np.array(target_xyz + target_rpy, dtype=float)
        W = np.diag([10.0, 10.0, 10.0, 1.0, 1.0, 1.0])

        q = np.array(self._initial_guess(), dtype=float)

        for _ in range(max_iter):
            current = np.array(self.forward(q.tolist()))
            error = target - current

            pos_error = np.linalg.norm(error[:3])
            if pos_error < pos_tol:
                self._q_prev = q.copy()
                return q.tolist(), True

            J = self._jacobian(q)
            WJ = W @ J
            WJT = WJ.T
            w_error = W @ error
            delta_q = WJT @ np.linalg.solve(WJ @ WJT + damping**2 * np.eye(6), w_error)

            q = q + delta_q
            self._clamp_joints(q)

        self._q_prev = q.copy()
        return q.tolist(), False

    def _initial_guess(self):
        """Use previous solution as seed for faster convergence."""
        if self._q_prev is not None:
            return self._q_prev.copy()
        return np.array([0.0, 1.0, -1.0, 0.0, 0.0, 0.0])

    def _jacobian(self, q, delta=0.001):
        """Numerical Jacobian via forward finite differences."""
        J = np.zeros((6, 6))
        f0 = np.array(self.forward(q.tolist()))
        for i in range(6):
            q_perturbed = q.copy()
            q_perturbed[i] += delta
            f1 = np.array(self.forward(q_perturbed.tolist()))
            J[:, i] = (f1 - f0) / delta
        return J

    def _clamp_joints(self, q):
        """Clamp joint angles to limits in-place."""
        for i in range(6):
            q[i] = max(JOINT_LIMITS_RAD[i][0], min(JOINT_LIMITS_RAD[i][1], q[i]))


if __name__ == "__main__":
    ik = PiperIK()

    print("Testing FK at home position [0,0,0,0,0,0]:")
    home = ik.forward([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    print(f"  End effector: x={home[0]:.1f} y={home[1]:.1f} z={home[2]:.1f} mm")
    print(f"  Orientation:  rx={home[3]:.1f} ry={home[4]:.1f} rz={home[5]:.1f} deg")

    print("\nTesting IK for SDK init position (56.1, 0, 213.3):")
    joints, ok = ik.solve([56.1, 0.0, 213.3])
    print(f"  Converged: {ok}")
    print(f"  Joints (deg): {[f'{math.degrees(j):.1f}' for j in joints]}")
    verify = ik.forward(joints)
    print(f"  Verify FK:    x={verify[0]:.1f} y={verify[1]:.1f} z={verify[2]:.1f}")

    print("\nTesting IK for extended position (200, 0, 300):")
    joints, ok = ik.solve([200.0, 0.0, 300.0])
    print(f"  Converged: {ok}")
    print(f"  Joints (deg): {[f'{math.degrees(j):.1f}' for j in joints]}")
    verify = ik.forward(joints)
    print(f"  Verify FK:    x={verify[0]:.1f} y={verify[1]:.1f} z={verify[2]:.1f}")

    print("\nTesting IK along a line (x=150→350, z=250):")
    for x in range(150, 360, 50):
        joints, ok = ik.solve([float(x), 0.0, 250.0])
        verify = ik.forward(joints)
        status = "OK" if ok else "FAIL"
        print(f"  Target x={x:3d} → actual x={verify[0]:.1f} z={verify[2]:.1f} [{status}]")
