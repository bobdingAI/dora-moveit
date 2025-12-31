#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
"""
Advanced IK Solver for LM3 6-DOF Robot
"""

import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass
from scipy.optimize import minimize, differential_evolution
from config.lm3_config import LM3Config


@dataclass
class IKRequest:
    """IK request containing target pose and optional seed"""
    target_position: np.ndarray
    target_orientation: Optional[np.ndarray] = None
    seed_joints: Optional[np.ndarray] = None
    orientation_type: str = "quaternion"


@dataclass
class IKResult:
    """Result of IK computation"""
    success: bool
    joint_positions: np.ndarray
    error: float
    iterations: int
    message: str = ""


class ForwardKinematicsLM3:
    """Forward kinematics for LM3 6-DOF robot"""

    def __init__(self):
        self.link_transforms = LM3Config.LINK_TRANSFORMS

    def compute_fk(self, joint_positions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Compute forward kinematics"""
        q = joint_positions

        def rot_z(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

        def rot_y(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

        def rot_x(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

        T = np.eye(4)

        for i, (joint_angle, link_tf) in enumerate(zip(q, self.link_transforms)):
            xyz = np.array(link_tf["xyz"])
            rpy = link_tf["rpy"]

            R_link = rot_z(rpy[2]) @ rot_y(rpy[1]) @ rot_x(rpy[0])
            T_link = np.eye(4)
            T_link[:3, :3] = R_link
            T_link[:3, 3] = xyz

            T = T @ T_link

            T_joint = np.eye(4)
            T_joint[:3, :3] = rot_z(joint_angle)
            T = T @ T_joint

        return T[:3, 3], T[:3, :3]

    def compute_jacobian(self, joint_positions: np.ndarray) -> np.ndarray:
        """Compute Jacobian matrix numerically"""
        jacobian = np.zeros((6, 6))
        delta = 1e-6

        pos0, rot0 = self.compute_fk(joint_positions)

        for i in range(6):
            q_delta = joint_positions.copy()
            q_delta[i] += delta
            pos1, rot1 = self.compute_fk(q_delta)
            jacobian[:3, i] = (pos1 - pos0) / delta
            jacobian[3:, i] = 0.0

        return jacobian


class TracIKSolverLM3:
    """TracIK-inspired solver for LM3"""

    def __init__(self):
        self.fk = ForwardKinematicsLM3()
        self.joint_limits_lower = LM3Config.JOINT_LOWER_LIMITS
        self.joint_limits_upper = LM3Config.JOINT_UPPER_LIMITS
        self.max_iterations = 500
        self.position_tolerance = 0.01

    def _objective_function(self, q: np.ndarray, target_pos: np.ndarray) -> float:
        """Objective function for optimization"""
        current_pos, _ = self.fk.compute_fk(q)
        return np.linalg.norm(target_pos - current_pos)

    def _position_error(self, q: np.ndarray, target_pos: np.ndarray) -> float:
        """Compute position error"""
        current_pos, _ = self.fk.compute_fk(q)
        return np.linalg.norm(target_pos - current_pos)

    def solve_jacobian(self, request: IKRequest) -> IKResult:
        """Solve IK using Jacobian method"""
        target_pos = request.target_position

        if request.seed_joints is not None:
            q = request.seed_joints.copy()
        else:
            q = LM3Config.SAFE_CONFIG.copy()

        step_size = 0.5
        damping = 0.01

        for iteration in range(self.max_iterations):
            q = np.asarray(q).flatten()[:6]

            current_pos, _ = self.fk.compute_fk(q)
            pos_error = target_pos - current_pos
            error_norm = np.linalg.norm(pos_error)

            if error_norm < self.position_tolerance:
                return IKResult(
                    success=True,
                    joint_positions=q,
                    error=error_norm,
                    iterations=iteration + 1,
                    message="Jacobian IK converged"
                )

            J = self.fk.compute_jacobian(q)[:3, :]
            JJT = J @ J.T
            J_pinv = J.T @ np.linalg.inv(JJT + damping * np.eye(3))
            dq = J_pinv @ pos_error
            q = q + step_size * dq
            q = np.clip(q, self.joint_limits_lower, self.joint_limits_upper)

        current_pos, _ = self.fk.compute_fk(q)
        final_error = np.linalg.norm(target_pos - current_pos)

        return IKResult(
            success=False,
            joint_positions=q,
            error=final_error,
            iterations=self.max_iterations,
            message=f"Jacobian IK failed, error={final_error:.6f}"
        )

    def solve_optimization(self, request: IKRequest) -> IKResult:
        """Solve IK using BFGS optimization"""
        target_pos = request.target_position

        if request.seed_joints is not None:
            q0 = request.seed_joints.copy()
        else:
            q0 = LM3Config.SAFE_CONFIG.copy()

        bounds = [(low, high) for low, high in zip(
            self.joint_limits_lower,
            self.joint_limits_upper
        )]

        result = minimize(
            fun=lambda q: self._objective_function(q, target_pos),
            x0=q0,
            method='L-BFGS-B',
            bounds=bounds,
            options={'maxiter': 500, 'ftol': 1e-6}
        )

        error = self._position_error(result.x, target_pos)
        success = error < self.position_tolerance

        return IKResult(
            success=success,
            joint_positions=result.x,
            error=error,
            iterations=result.nit,
            message="BFGS optimization completed" if success else f"BFGS failed, error={error:.6f}"
        )

    def solve_multistart(self, request: IKRequest, num_attempts: int = 5) -> IKResult:
        """Solve IK using multiple random starts"""
        best_result = None
        best_error = float('inf')

        for attempt in range(num_attempts):
            if attempt == 0 and request.seed_joints is not None:
                seed = request.seed_joints.copy()
            else:
                seed = np.random.uniform(
                    self.joint_limits_lower,
                    self.joint_limits_upper
                )

            request_copy = IKRequest(
                target_position=request.target_position,
                target_orientation=request.target_orientation,
                seed_joints=seed,
                orientation_type=request.orientation_type
            )

            result = self.solve_jacobian(request_copy)

            if result.success:
                return result

            if result.error < best_error:
                best_error = result.error
                best_result = result

        return best_result

    def solve(self, request: IKRequest) -> IKResult:
        """Main solve method - tries multiple strategies"""
        result = self.solve_jacobian(request)
        if result.success:
            return result

        result = self.solve_optimization(request)
        if result.success:
            return result

        result = self.solve_multistart(request, num_attempts=3)
        return result


class DifferentialEvolutionIKSolverLM3:
    """Differential Evolution IK solver for LM3"""

    def __init__(self):
        self.fk = ForwardKinematicsLM3()
        self.joint_limits_lower = LM3Config.JOINT_LOWER_LIMITS
        self.joint_limits_upper = LM3Config.JOINT_UPPER_LIMITS
        self.position_tolerance = 0.01

    def _objective_function(self, q: np.ndarray, target_pos: np.ndarray) -> float:
        """Objective function"""
        current_pos, _ = self.fk.compute_fk(q)
        return np.linalg.norm(target_pos - current_pos)

    def solve(self, request: IKRequest) -> IKResult:
        """Solve IK using differential evolution"""
        target_pos = request.target_position

        bounds = [(low, high) for low, high in zip(
            self.joint_limits_lower,
            self.joint_limits_upper
        )]

        result = differential_evolution(
            func=lambda q: self._objective_function(q, target_pos),
            bounds=bounds,
            maxiter=300,
            popsize=15,
            tol=1e-6,
            atol=1e-6
        )

        error = result.fun
        success = error < self.position_tolerance

        return IKResult(
            success=success,
            joint_positions=result.x,
            error=error,
            iterations=result.nit,
            message="DE optimization completed" if success else f"DE failed, error={error:.6f}"
        )
