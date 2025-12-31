#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
"""
Lebai LM3 Real Robot Control Node
Uses lebai_sdk to control physical LM3 robot
"""

import numpy as np
import pyarrow as pa
from dora import Node
from config.lm3_config import LM3Config

try:
    import lebai_sdk
    LEBAI_SDK_AVAILABLE = True
except ImportError:
    LEBAI_SDK_AVAILABLE = False
    print("[LM3-Robot] WARNING: lebai_sdk not available, running in mock mode")


class LM3RobotNode:
    """Real LM3 robot control via Lebai SDK"""

    def __init__(self, robot_ip: str = "192.168.1.200"):
        self.robot_ip = robot_ip
        self.robot = None
        self.current_joints = LM3Config.SAFE_CONFIG.copy()
        self.target_joints = LM3Config.SAFE_CONFIG.copy()
        self.is_connected = False
        self.mock_mode = not LEBAI_SDK_AVAILABLE

        if not self.mock_mode:
            self._connect()
        else:
            print("[LM3-Robot] Running in MOCK mode (no real robot)")

    def _connect(self):
        """Connect to LM3 robot"""
        try:
            lebai_sdk.init()
            self.robot = lebai_sdk.connect(self.robot_ip, False)
            self.robot.start_sys()
            self.is_connected = True
            print(f"[LM3-Robot] Connected to LM3 at {self.robot_ip}")

            # Read initial joint state
            data = self.robot.get_kin_data()
            self.current_joints = np.array(data["actual_joint_pose"])
            print(f"[LM3-Robot] Initial joints: {self.current_joints}")

        except Exception as e:
            print(f"[LM3-Robot] Connection failed: {e}")
            self.is_connected = False
            self.mock_mode = True

    def read_joint_state(self) -> np.ndarray:
        """Read current joint positions from robot"""
        if self.mock_mode:
            return self.current_joints

        try:
            data = self.robot.get_kin_data()
            self.current_joints = np.array(data["actual_joint_pose"])
            return self.current_joints
        except Exception as e:
            print(f"[LM3-Robot] Error reading joints: {e}")
            return self.current_joints

    def send_joint_command(self, target_joints: np.ndarray):
        """Send joint position command to robot"""
        self.target_joints = target_joints.copy()

        if self.mock_mode:
            # Simulate motion
            self.current_joints = self.target_joints.copy()
            return

        try:
            # Use move_pvat for smooth motion
            # Parameters: joint_position, velocity, acceleration, time
            t = 0.2  # Motion time in seconds
            self.robot.move_pvat(
                list(target_joints),
                [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],  # Velocity limits
                [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],  # Acceleration limits
                t
            )
        except Exception as e:
            print(f"[LM3-Robot] Error sending command: {e}")

    def disconnect(self):
        """Disconnect from robot"""
        if self.is_connected and not self.mock_mode:
            try:
                self.robot.stop_sys()
                print("[LM3-Robot] Disconnected from robot")
            except:
                pass


def main():
    """Main entry point for LM3 robot node"""
    print("=== LM3 Real Robot Control Node ===")

    # Get robot IP from environment or use default
    robot_ip = os.getenv("LEBAI_IP", "192.168.1.200")

    node = Node()
    robot_node = LM3RobotNode(robot_ip=robot_ip)

    print("[LM3-Robot] Node started, waiting for commands...")

    # Send initial joint state
    node.send_output(
        "joint_positions",
        pa.array(robot_node.current_joints, type=pa.float32())
    )

    try:
        for event in node:
            event_type = event["type"]

            if event_type == "INPUT":
                input_id = event["id"]

                if input_id == "tick":
                    # Read current joint state
                    joints = robot_node.read_joint_state()

                    # Send joint state
                    node.send_output(
                        "joint_positions",
                        pa.array(joints, type=pa.float32())
                    )

                elif input_id == "joint_command":
                    # Receive joint command
                    target_joints = event["value"].to_numpy()

                    if len(target_joints) == 6:
                        robot_node.send_joint_command(target_joints)
                    else:
                        print(f"[LM3-Robot] Invalid command length: {len(target_joints)}")

            elif event_type == "STOP":
                print("[LM3-Robot] Stopping...")
                break

    finally:
        robot_node.disconnect()


if __name__ == "__main__":
    main()
