#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
"""
Trajectory Executor for Dora-MoveIt + MuJoCo
============================================

Executes planned trajectories by sending joint commands to robot.
Uses quintic polynomial interpolation for smooth motion.
"""

import json
import numpy as np
import pyarrow as pa
from typing import List, Optional
from dora import Node


class TrajectoryExecutor:
    """Executes motion trajectories using quintic polynomial interpolation"""

    def __init__(self, num_joints: int = 7):
        self.num_joints = num_joints
        self.trajectory: List[np.ndarray] = []
        self.current_waypoint_idx = 0
        self.prev_waypoint: Optional[np.ndarray] = None

        self.interpolation_progress = 0.0
        self.interpolation_speed = 0.03  # 3%每tick,匹配30%机器人速度和150ms tick

        self.is_executing = False
        self.execution_count = 0
        self.current_trajectory_hash: Optional[int] = None

        self.current_joints: Optional[np.ndarray] = None
        self.last_command: Optional[np.ndarray] = None

    def cubic_interpolate(self, q_start: np.ndarray, q_end: np.ndarray, t: float) -> np.ndarray:
        """
        五次多项式插值（Quintic Polynomial Interpolation）

        使用边界条件：起始和结束的速度、加速度都为0，保证最平滑的运动

        Args:
            q_start: 起始关节位置
            q_end: 结束关节位置
            t: 归一化时间 [0, 1]

        Returns:
            插值后的关节位置
        """
        t = np.clip(t, 0.0, 1.0)

        # 五次多项式系数 (边界条件: 位置、速度、加速度在起点和终点都确定)
        # q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        # 边界条件: q(0)=q_start, q(1)=q_end, q'(0)=0, q'(1)=0, q''(0)=0, q''(1)=0
        # 求解得到:
        h0 = 1 - 10*t**3 + 15*t**4 - 6*t**5   # 起始位置权重
        h1 = 10*t**3 - 15*t**4 + 6*t**5        # 结束位置权重

        return h0 * q_start + h1 * q_end

    def set_trajectory(self, trajectory: List[np.ndarray], trajectory_hash: int):
        """Set a new trajectory to execute"""
        if self.current_trajectory_hash == trajectory_hash:
            return

        self.trajectory = trajectory
        self.current_trajectory_hash = trajectory_hash
        self.interpolation_progress = 0.0
        self.is_executing = True
        self.execution_count += 1

        if len(trajectory) > 0:
            self.prev_waypoint = trajectory[0]
            self.current_waypoint_idx = 1 if len(trajectory) > 1 else 0
            self.last_command = trajectory[0].copy()
            print(f"[Executor] New trajectory with {len(trajectory)} waypoints (quintic interpolation)")

    def update_current_joints(self, joints: np.ndarray):
        """Update current joint positions from robot"""
        self.current_joints = joints[:self.num_joints].copy()

    def step(self) -> Optional[np.ndarray]:
        """
        Execute one step with quintic polynomial interpolation.
        ALWAYS output current joint state when idle.
        """

        # =========================
        # IDLE / HOLD MODE
        # =========================
        if not self.is_executing or len(self.trajectory) == 0:
            if self.current_joints is not None:
                return self.current_joints.copy()
            return self.last_command

        if self.prev_waypoint is None:
            return self.current_joints.copy() if self.current_joints is not None else self.last_command

        # =========================
        # EXECUTION MODE
        # =========================
        target = self.trajectory[self.current_waypoint_idx]
        self.interpolation_progress += self.interpolation_speed

        if self.interpolation_progress >= 1.0:
            self.prev_waypoint = target
            self.current_waypoint_idx += 1
            self.interpolation_progress = 0.0

            # ===== Trajectory finished =====
            if self.current_waypoint_idx >= len(self.trajectory):
                self.is_executing = False
                print(f"[Executor] Trajectory #{self.execution_count} complete!")

                # CRITICAL FIX:
                # Do NOT output last waypoint
                # Hold current real joint state
                if self.current_joints is not None:
                    self.last_command = self.current_joints.copy()
                    return self.current_joints.copy()

                return self.last_command

            target = self.trajectory[self.current_waypoint_idx]

        # 使用五次多项式插值
        t = min(self.interpolation_progress, 1.0)
        command = self.cubic_interpolate(self.prev_waypoint, target, t)
        self.last_command = command.copy()
        return command

    def get_status(self) -> dict:
        return {
            "is_executing": self.is_executing,
            "execution_count": self.execution_count,
            "current_waypoint": self.current_waypoint_idx,
            "total_waypoints": len(self.trajectory),
            "progress": self.interpolation_progress
        }


def main():
    print("=== Dora-MoveIt Trajectory Executor (Quintic Interpolation) ===")

    node = Node()
    executor = TrajectoryExecutor(num_joints=7)

    from config.robot_config import GEN72Config
    executor.current_joints = GEN72Config.SAFE_CONFIG.copy()
    executor.last_command = GEN72Config.SAFE_CONFIG.copy()
    print(f"Initialized with safe config: {executor.current_joints[:6]}...")

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]

            if input_id == "trajectory":
                traj_flat = event["value"].to_numpy()
                metadata = event.get("metadata", {})
                num_waypoints = metadata.get("num_waypoints", len(traj_flat) // 7)
                num_joints = metadata.get("num_joints", 7)

                trajectory = traj_flat.reshape(num_waypoints, num_joints)
                trajectory_list = [trajectory[i] for i in range(num_waypoints)]

                if executor.current_joints is not None:
                    trajectory_list.insert(0, executor.current_joints.copy())

                traj_hash = hash(traj_flat.tobytes())
                executor.set_trajectory(trajectory_list, traj_hash)

            elif input_id == "joint_positions":
                executor.update_current_joints(event["value"].to_numpy())

            elif input_id == "tick":
                command = executor.step()

                if command is not None:
                    node.send_output(
                        "joint_commands",
                        pa.array(command, type=pa.float32())
                    )

                status_bytes = json.dumps(executor.get_status()).encode("utf-8")
                node.send_output(
                    "execution_status",
                    pa.array(list(status_bytes), type=pa.uint8())
                )

        elif event["type"] == "STOP":
            print("Trajectory executor stopping...")
            break


if __name__ == "__main__":
    main()
