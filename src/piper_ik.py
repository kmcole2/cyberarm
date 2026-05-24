#!/usr/bin/env python3
"""
Numerical Inverse Kinematics for the Piper arm.

Uses the SDK's forward kinematics as a black box and solves via
damped least squares on the position Jacobian (3×6). Converges in
2-5 iterations for typical trajectories.
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

    def solve(self, target_xyz, max_iter=50, pos_tol=2.0):
        """
        Solve IK for a target XYZ position.

        Only minimizes position error (3 constraints, 6 joints).
        The redundancy naturally resolves toward the initial guess
        configuration, keeping wrist joints near zero.

        Args:
            target_xyz: [x, y, z] in mm
            max_iter: iteration cap
            pos_tol: position convergence threshold in mm

        Returns:
            (joints_rad, converged): list of 6 joint angles in radians, success bool
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

    # Alias for backward compat
    solve_position_only = solve

    def _get_initial_guess(self, target_xyz):
        """Use previous converged solution if nearby, else start from a neutral pose."""
        if self._prev_converged and self._q_prev is not None:
            prev_pos = np.array(self.forward(self._q_prev.tolist())[:3])
            dist = np.linalg.norm(prev_pos - np.array(target_xyz))
            if dist < 150.0:
                return self._q_prev.copy()

        return np.array([0.0, 0.8, -0.8, 0.0, 0.0, 0.0])

    def _position_jacobian(self, q, delta=0.001):
        """Numerical position Jacobian (3×6) via forward differences."""
        J = np.zeros((3, 6))
        f0 = np.array(self.forward(q.tolist())[:3])
        for i in range(6):
            q_p = q.copy()
            q_p[i] += delta
            f1 = np.array(self.forward(q_p.tolist())[:3])
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

    print("\nTesting problem points (non-zero Y):")
    test_points = [
        (141.4, -34.0, 196.1),
        (142.5, -34.0, 194.9),
        (143.5, -33.9, 193.9),
        (144.5, -33.8, 192.9),
        (145.5, -33.7, 192.0),
    ]
    for x, y, z in test_points:
        joints, ok = ik.solve([x, y, z])
        verify = ik.forward(joints)
        err = math.sqrt((verify[0]-x)**2 + (verify[1]-y)**2 + (verify[2]-z)**2)
        status = "OK" if ok else f"FAIL err={err:.1f}mm"
        print(f"  ({x:.1f}, {y:.1f}, {z:.1f}) → [{status}]")

    print("\nTesting diverse positions (fresh solver, no warm-start):")
    ik2 = PiperIK()
    diverse_points = [
        (200.0, 50.0, 300.0),
        (250.0, -80.0, 200.0),
        (100.0, 100.0, 350.0),
        (300.0, 0.0, 250.0),
        (50.0, -50.0, 400.0),
        (180.0, 60.0, 150.0),
        (350.0, 0.0, 250.0),
        (56.1, 0.0, 213.3),
    ]
    for x, y, z in diverse_points:
        joints, ok = ik2.solve([x, y, z])
        verify = ik2.forward(joints)
        err = math.sqrt((verify[0]-x)**2 + (verify[1]-y)**2 + (verify[2]-z)**2)
        status = "OK" if ok else f"FAIL err={err:.1f}mm"
        print(f"  ({x:.1f}, {y:.1f}, {z:.1f}) → [{status}]")

    print("\nBenchmark:")
    import time
    ik3 = PiperIK()
    start = time.time()
    count = 0
    for x in range(100, 350, 5):
        for y in range(-80, 80, 40):
            ik3.solve([float(x), float(y), 250.0])
            count += 1
    elapsed = time.time() - start
    print(f"  {count} solves in {elapsed*1000:.0f}ms ({elapsed/count*1000:.2f}ms each, {1/(elapsed/count):.0f} Hz)")
