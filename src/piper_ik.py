#!/usr/bin/env python3
"""
Numerical Inverse Kinematics for the Piper arm.

Uses the SDK's forward kinematics as a black box and solves via
damped least squares. Supports both position-only (3-DOF) and
full pose (6-DOF position + orientation) solving.
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
        self._prev_converged = False

    def forward(self, joints_rad):
        """FK wrapper: joints (rad) -> [x, y, z, rx, ry, rz] (mm, degrees)."""
        result = self.fk.CalFK(joints_rad)
        return result[-1]

    def solve(self, target_xyz, target_rpy=None, max_iter=80, pos_tol=2.0, ori_tol=5.0):
        """
        Solve full 6-DOF IK for position + orientation.

        Uses a two-phase approach:
        1. Solve position-only first (robust, underdetermined)
        2. Adjust orientation via null-space of position Jacobian
           (only moves joints in ways that don't disturb position)

        Args:
            target_xyz: [x, y, z] in mm
            target_rpy: [rx, ry, rz] in degrees (default: [0, 85, 0])
            max_iter: iteration cap (split between phases)
            pos_tol: position convergence threshold in mm
            ori_tol: orientation convergence threshold in degrees

        Returns:
            (joints_rad, converged): list of 6 joint angles in radians, success bool
        """
        if target_rpy is None:
            target_rpy = [0.0, 85.0, 0.0]

        # Phase 1: solve position using robust 3×6 method
        target_pos = np.array(target_xyz, dtype=float)
        target_ori = np.array(target_rpy, dtype=float)
        q = self._get_initial_guess(target_xyz)

        for _ in range(max_iter // 2):
            cur = np.array(self.forward(q.tolist()))
            pos_error = target_pos - cur[:3]
            if np.linalg.norm(pos_error) < pos_tol:
                break
            Jp = self._position_jacobian(q)
            delta_q = Jp.T @ np.linalg.solve(Jp @ Jp.T + 0.01**2 * np.eye(3), pos_error)
            q = q + delta_q
            self._clamp_joints(q)

        # Phase 2: adjust orientation in the null space of position
        for _ in range(max_iter // 2):
            cur = np.array(self.forward(q.tolist()))
            pos_err = np.linalg.norm(target_pos - cur[:3])
            ori_err = np.linalg.norm(target_ori - cur[3:])

            if pos_err < pos_tol and ori_err < ori_tol:
                self._q_prev = q.copy()
                self._prev_converged = True
                return q.tolist(), True

            Jp = self._position_jacobian(q)

            # Maintain position: correct any drift
            pos_error = target_pos - cur[:3]
            Jp_pinv = Jp.T @ np.linalg.inv(Jp @ Jp.T + 0.01**2 * np.eye(3))
            delta_q_pos = Jp_pinv @ pos_error

            # Null-space projector: I - J+ @ J
            N = np.eye(6) - Jp_pinv @ Jp

            # Orientation gradient in null space
            Jo = self._orientation_jacobian(q)
            ori_error = target_ori - cur[3:]
            # Desired joint step for orientation (unconstrained)
            delta_q_ori_desired = Jo.T @ np.linalg.solve(Jo @ Jo.T + 0.1**2 * np.eye(3), ori_error)
            # Project into null space
            delta_q_null = N @ delta_q_ori_desired

            delta_q = delta_q_pos + delta_q_null

            # Limit step size
            step = np.linalg.norm(delta_q)
            if step > 0.2:
                delta_q = delta_q * (0.2 / step)

            q = q + delta_q
            self._clamp_joints(q)

        # Accept as converged if position is good (orientation is best-effort)
        final = np.array(self.forward(q.tolist()))
        pos_err = np.linalg.norm(final[:3] - target_pos)

        self._q_prev = q.copy()
        self._prev_converged = pos_err < pos_tol
        return q.tolist(), pos_err < pos_tol

    def solve_position_only(self, target_xyz, max_iter=50, pos_tol=2.0):
        """
        Solve IK for position only (ignores orientation).

        Uses the 3×6 position Jacobian. More robust when orientation
        doesn't matter.
        """
        target_pos = np.array(target_xyz, dtype=float)
        q = self._get_initial_guess(target_xyz)

        for _ in range(max_iter):
            cur_pos = np.array(self.forward(q.tolist())[:3])
            error = target_pos - cur_pos
            err_norm = np.linalg.norm(error)

            if err_norm < pos_tol:
                self._q_prev = q.copy()
                self._prev_converged = True
                return q.tolist(), True

            Jp = self._position_jacobian(q)
            lam = 0.01
            delta_q = Jp.T @ np.linalg.solve(Jp @ Jp.T + lam**2 * np.eye(3), error)

            q = q + delta_q
            self._clamp_joints(q)

        self._q_prev = q.copy()
        self._prev_converged = False
        return q.tolist(), False

    def _get_initial_guess(self, target_xyz):
        """Use previous converged solution if nearby, else start from a neutral pose."""
        if self._prev_converged and self._q_prev is not None:
            prev_pos = np.array(self.forward(self._q_prev.tolist())[:3])
            dist = np.linalg.norm(prev_pos - np.array(target_xyz))
            if dist < 150.0:
                return self._q_prev.copy()

        return np.array([0.0, 0.8, -0.8, 0.0, 0.0, 0.0])

    def _position_jacobian(self, q, delta=0.001):
        """Numerical position Jacobian (3×6)."""
        J = np.zeros((3, 6))
        f0 = np.array(self.forward(q.tolist())[:3])
        for i in range(6):
            q_p = q.copy()
            q_p[i] += delta
            f1 = np.array(self.forward(q_p.tolist())[:3])
            J[:, i] = (f1 - f0) / delta
        return J

    def _orientation_jacobian(self, q, delta=0.001):
        """Numerical orientation Jacobian (3×6)."""
        J = np.zeros((3, 6))
        f0 = np.array(self.forward(q.tolist())[3:])
        for i in range(6):
            q_p = q.copy()
            q_p[i] += delta
            f1 = np.array(self.forward(q_p.tolist())[3:])
            J[:, i] = (f1 - f0) / delta
        return J

    def _full_jacobian(self, q, delta=0.001):
        """Numerical full Jacobian (6×6) for position + orientation."""
        J = np.zeros((6, 6))
        f0 = np.array(self.forward(q.tolist()))
        for i in range(6):
            q_p = q.copy()
            q_p[i] += delta
            f1 = np.array(self.forward(q_p.tolist()))
            J[:, i] = (f1 - f0) / delta
        return J

    def _clamp_joints(self, q):
        """Clamp joint angles to limits in-place."""
        for i in range(6):
            q[i] = max(JOINT_LIMITS_RAD[i][0], min(JOINT_LIMITS_RAD[i][1], q[i]))


if __name__ == "__main__":
    ik = PiperIK()

    print("Testing FK at home [0,0,0,0,0,0]:")
    home = ik.forward([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    print(f"  pos=({home[0]:.1f}, {home[1]:.1f}, {home[2]:.1f})  ori=({home[3]:.1f}, {home[4]:.1f}, {home[5]:.1f})")

    print("\nTesting full 6-DOF solve (position + orientation):")
    test_cases = [
        ([141.4, -34.0, 196.1], [0.0, 85.0, 0.0]),
        ([200.0, 50.0, 300.0], [0.0, 85.0, 0.0]),
        ([250.0, -80.0, 200.0], [0.0, 45.0, 0.0]),
        ([300.0, 0.0, 250.0], [10.0, 70.0, -10.0]),
        ([150.0, 0.0, 350.0], [0.0, 90.0, 0.0]),
    ]
    for xyz, rpy in test_cases:
        joints, ok = ik.solve(xyz, rpy)
        v = ik.forward(joints)
        pos_err = math.sqrt(sum((v[i]-xyz[i])**2 for i in range(3)))
        ori_err = math.sqrt(sum((v[i+3]-rpy[i])**2 for i in range(3)))
        status = "OK" if ok else f"pos_err={pos_err:.1f}mm ori_err={ori_err:.1f}°"
        print(f"  xyz={xyz} rpy={rpy} → [{status}]")

    print("\nTesting position-only solve:")
    ik2 = PiperIK()
    for x, y, z in [(141.4, -34.0, 196.1), (250.0, -80.0, 200.0), (350.0, 0.0, 250.0)]:
        joints, ok = ik2.solve_position_only([x, y, z])
        v = ik2.forward(joints)
        err = math.sqrt((v[0]-x)**2 + (v[1]-y)**2 + (v[2]-z)**2)
        print(f"  ({x}, {y}, {z}) → [{'OK' if ok else f'FAIL err={err:.1f}mm'}]")
