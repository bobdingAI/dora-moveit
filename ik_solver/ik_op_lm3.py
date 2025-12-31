#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
"""
Inverse Kinematics Operator for LM3 6-DOF Robot
"""

import json
import numpy as np
import pyarrow as pa
from typing import Optional, Tuple
from dora import Node
from config.lm3_config import LM3Config
from ik_solver.advanced_ik_solver_lm3 import TracIKSolverLM3, DifferentialEvolutionIKSolverLM3, IKRequest, IKResult


class NumericalIKSolverLM3:
    """Numerical IK solver for LM3 6-DOF robot"""

    def __init__(self):
        self.num_joints = 6
        self.max_iterations = 500
        self.position_tolerance = 0.01
        self.step_size = 0.5
        self.joint_limits_lower = LM3Config.JOINT_LOWER_LIMITS
        self.joint_limits_upper = LM3Config.JOINT_UPPER_LIMITS
        self.link_transforms = LM3Config.LINK_TRANSFORMS

    def forward_kinematics(self, joint_positions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Compute forward kinematics for LM3"""
        q = joint_positions

        def rot_z(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

        def rot_x(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

        T = np.eye(4)

        for i, (joint_angle, link_tf) in enumerate(zip(q, self.link_transforms)):
            xyz = np.array(link_tf["xyz"])
            rpy = link_tf["rpy"]

            R_link = rot_z(rpy[2]) @ rot_x(rpy[0])
            T_link = np.eye(4)
            T_link[:3, :3] = R_link
            T_link[:3, 3] = xyz

            T = T @ T_link

            T_joint = np.eye(4)
            T_joint[:3, :3] = rot_z(joint_angle)
            T = T @ T_joint

        return T[:3, 3], T[:3, :3]

    def compute_jacobian(self, joint_positions: np.ndarray) -> np.ndarray:
        """Compute Jacobian matrix for LM3"""
        jacobian = np.zeros((6, self.num_joints))
        delta = 1e-6

        pos0, rot0 = self.forward_kinematics(joint_positions)

        for i in range(self.num_joints):
            q_delta = joint_positions.copy()
            q_delta[i] += delta
            pos1, rot1 = self.forward_kinematics(q_delta)
            jacobian[:3, i] = (pos1 - pos0) / delta
            jacobian[3:, i] = 0.0

        return jacobian

    def solve(self, request: IKRequest) -> IKResult:
        """Solve IK for LM3"""
        target_pos = request.target_position

        if request.seed_joints is not None:
            q = request.seed_joints.copy()
        else:
            q = LM3Config.SAFE_CONFIG.copy()

        for iteration in range(self.max_iterations):
            q = np.asarray(q).flatten()[:self.num_joints]
            current_pos, current_rot = self.forward_kinematics(q)
            pos_error = target_pos - current_pos
            error_norm = np.linalg.norm(pos_error)

            if error_norm < self.position_tolerance:
                return IKResult(
                    success=True,
                    joint_positions=q,
                    error=error_norm,
                    iterations=iteration + 1,
                    message="IK converged successfully"
                )

            J = self.compute_jacobian(q)[:3, :]
            damping = 0.001
            JJT = J @ J.T
            J_pinv = J.T @ np.linalg.inv(JJT + damping * np.eye(3))
            dq = J_pinv @ pos_error
            q = q + self.step_size * dq
            q = np.clip(q, self.joint_limits_lower, self.joint_limits_upper)

        current_pos, _ = self.forward_kinematics(q)
        final_error = np.linalg.norm(target_pos - current_pos)

        return IKResult(
            success=False,
            joint_positions=q,
            error=final_error,
            iterations=self.max_iterations,
            message=f"IK failed to converge. Error: {final_error:.6f}"
        )


class IKOperatorLM3:
    """Dora operator for LM3 Inverse Kinematics"""

    def __init__(self, solver_type: str = "tracik"):
        if solver_type == "tracik":
            self.solver = TracIKSolverLM3()
            print("[IK-LM3] Using TracIK solver (6-DOF)")
        elif solver_type == "de":
            self.solver = DifferentialEvolutionIKSolverLM3()
            print("[IK-LM3] Using Differential Evolution solver (6-DOF)")
        else:
            self.solver = NumericalIKSolverLM3()
            print("[IK-LM3] Using simple numerical solver (6-DOF)")

        self.solver_type = solver_type
        self.current_joints: Optional[np.ndarray] = None
        self.request_count = 0

    def process_joint_state(self, joint_positions: np.ndarray):
        """Update current joint state for seeding"""
        self.current_joints = joint_positions.copy()

    def process_ik_request(self, pose_data: np.ndarray) -> Tuple[Optional[np.ndarray], dict]:
        """Process an IK request"""
        self.request_count += 1

        if len(pose_data) == 6:
            position = pose_data[:3]
            orientation = pose_data[3:6]
            orientation_type = "rpy"
        elif len(pose_data) == 7:
            position = pose_data[:3]
            orientation = pose_data[3:7]
            orientation_type = "quaternion"
        else:
            return None, {
                "success": False,
                "error": f"Invalid pose length: {len(pose_data)}",
                "request_id": self.request_count
            }

        request = IKRequest(
            target_position=position,
            target_orientation=orientation,
            seed_joints=self.current_joints,
            orientation_type=orientation_type
        )

        result = self.solver.solve(request)

        status = {
            "success": result.success,
            "error": float(result.error),
            "iterations": result.iterations,
            "message": result.message,
            "request_id": self.request_count,
            "target_position": position.tolist()
        }

        if result.success:
            return result.joint_positions, status
        else:
            return None, status


def main():
    """Main entry point for Dora IK operator"""
    print("=== Dora-MoveIt IK Operator (LM3 6-DOF) ===")

    node = Node()
    ik_op = IKOperatorLM3(solver_type="tracik")

    print("IK operator started, waiting for requests...")

    for event in node:
        event_type = event["type"]

        if event_type == "INPUT":
            input_id = event["id"]

            if input_id == "joint_state":
                joints = event["value"].to_numpy()
                ik_op.process_joint_state(joints)

            elif input_id == "ik_request":
                pose = event["value"].to_numpy()
                print(f"[IK-LM3] Request #{ik_op.request_count + 1}: target={pose[:3]}")

                solution, status = ik_op.process_ik_request(pose)

                status_bytes = json.dumps(status).encode('utf-8')
                node.send_output("ik_status", pa.array(list(status_bytes), type=pa.uint8()))

                if solution is not None:
                    node.send_output(
                        "ik_solution",
                        pa.array(solution, type=pa.float32()),
                        metadata={"encoding": "jointstate", "success": True}
                    )
                    print(f"[IK-LM3] SUCCESS: Solution found, error={status['error']:.6f}")
                else:
                    print(f"[IK-LM3] FAILED: {status['message']}")

        elif event_type == "STOP":
            print("IK operator stopping...")
            break


if __name__ == "__main__":
    main()
