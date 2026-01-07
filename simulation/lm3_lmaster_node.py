#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
"""
LM3 L-Master Simulation Node (Adapted from lebai_driver_node.py)

Adapted for integration with dora-moveit control system.
Supports tick-based state updates and trajectory tracking.

Inputs:
    - tick: Timer trigger for periodic state updates
    - control_input: Joint commands from trajectory_executor
    - command: Control commands (home, safe, reset, get_state)

Outputs:
    - joint_positions: Current joint positions (6 values)
    - joint_velocities: Current joint velocities (6 values)
    - status: Node status (idle, moving, error)
    - error: Error messages (JSON)
"""

import json
import numpy as np
import pyarrow as pa
from dora import Node

try:
    import lebai_sdk
    LEBAI_SDK_AVAILABLE = True
except ImportError:
    LEBAI_SDK_AVAILABLE = False
    print("[LM3-LMaster] WARNING: lebai_sdk not available")


class LM3LMasterNode:
    """L-Master simulation node for LM3 robot control."""

    def __init__(self):
        self.arm = None
        self.ip = os.getenv("LEBAI_IP", "127.0.0.1")
        self.simulation = os.getenv("LEBAI_SIMULATION", "true").lower() == "true"
        self.acceleration = float(os.getenv("LEBAI_ACCELERATION", "0.6"))
        self.velocity = float(os.getenv("LEBAI_VELOCITY", "0.3"))
        self.connected = False

        # Safe position
        self.safe_position = [0.0, -0.5, 0.3, 0.0, 0.3, 0.0]
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Track last command to avoid redundant moves
        self.last_target = None
        self.position_threshold = 0.05  # 50 millirad (~2.86 degrees)

    def connect(self) -> dict:
        """Connect to L-Master simulator."""
        if not LEBAI_SDK_AVAILABLE:
            return {"success": False, "error": "lebai_sdk not available"}

        try:
            lebai_sdk.init()
            self.arm = lebai_sdk.connect(self.ip, self.simulation)

            if not self.arm.is_connected():
                return {"success": False, "error": "Connection failed: Check L-Master container"}

            self.arm.start_sys()
            self.connected = True
            return {"success": True, "message": f"Connected to L-Master at {self.ip}"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def disconnect(self) -> dict:
        """Disconnect from simulator."""
        try:
            if self.arm and self.connected:
                self.arm.stop_sys()
                self.connected = False
            return {"success": True, "message": "Disconnected"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def get_current_state(self) -> dict:
        """Get current joint positions and velocities."""
        try:
            if not self.connected:
                return {"success": False, "error": "Not connected"}

            # Get kinematic data
            data = self.arm.get_kin_data()
            joints = list(data["actual_joint_pose"])
            velocities = list(data.get("actual_joint_velocity", [0.0] * 6))

            return {
                "success": True,
                "joints": joints,
                "velocities": velocities
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    def move_joints(self, target_joints: list, wait: bool = False) -> dict:
        """Move robot to target joint positions."""
        try:
            if not self.connected:
                return {"success": False, "error": "Not connected"}

            if len(target_joints) != 6:
                return {"success": False, "error": f"Expected 6 joint values, got {len(target_joints)}"}

            # Convert to Python float (lebai_sdk doesn't accept numpy float32)
            target_joints = [float(x) for x in target_joints]

            # Use move_pvat with 50ms timing to match tick frequency
            # This reduces double interpolation conflict
            self.arm.move_pvat(
                target_joints,
                [self.velocity] * 6,      # Velocity limits for each joint
                [self.acceleration] * 6,  # Acceleration limits for each joint
                0.05                      # 50ms movement time (matches tick)
            )

            if wait:
                self.arm.wait_move()

            return {"success": True, "message": "Movement command sent"}
        except Exception as e:
            return {"success": False, "error": str(e)}


def main():
    """Main entry point for L-Master simulation node."""
    print("=== LM3 L-Master Simulation Node ===")

    node = Node()
    driver = LM3LMasterNode()

    # Auto-connect on startup
    connect_result = driver.connect()
    print(f"[LM3-LMaster] Connection: {connect_result}")

    if not connect_result.get("success"):
        node.send_output("error", pa.array([json.dumps(connect_result)]))
        node.send_output("status", pa.array(["error"]))
        print("[LM3-LMaster] Failed to connect, exiting")
        return

    # Send initial joint state
    initial_state = driver.get_current_state()
    if initial_state.get("success"):
        joints = np.array(initial_state["joints"])
        velocities = np.array(initial_state["velocities"])
        node.send_output("joint_positions", pa.array(joints, type=pa.float32()))
        node.send_output("joint_velocities", pa.array(velocities, type=pa.float32()))
        print(f"[LM3-LMaster] Initial joints: {joints}")

    # Send initial status
    node.send_output("status", pa.array(["idle"]))
    print("[LM3-LMaster] Node started, waiting for commands...")

    try:
        for event in node:
            event_type = event["type"]

            if event_type == "INPUT":
                event_id = event["id"]

                if event_id == "tick":
                    # Periodic state update
                    result = driver.get_current_state()

                    if result["success"]:
                        joints = np.array(result["joints"])
                        velocities = np.array(result["velocities"])

                        node.send_output("joint_positions", pa.array(joints, type=pa.float32()))
                        node.send_output("joint_velocities", pa.array(velocities, type=pa.float32()))
                        node.send_output("status", pa.array(["idle"]))
                    else:
                        node.send_output("status", pa.array(["error"]))
                        node.send_output("error", pa.array([json.dumps(result)]))

                elif event_id == "control_input":
                    # Receive joint command from trajectory_executor
                    try:
                        target = event["value"].to_numpy()

                        # Check if position changed significantly
                        if driver.last_target is not None:
                            diff = np.abs(target - driver.last_target)
                            if np.all(diff < driver.position_threshold):
                                # Position unchanged, skip command
                                continue

                        # Send command (non-blocking)
                        result = driver.move_joints(list(target), wait=False)
                        driver.last_target = target.copy()

                        if result["success"]:
                            node.send_output("status", pa.array(["moving"]))
                        else:
                            node.send_output("status", pa.array(["error"]))
                            node.send_output("error", pa.array([json.dumps(result)]))

                    except Exception as e:
                        error_msg = {"success": False, "error": f"Parse error: {str(e)}"}
                        print(f"[LM3-LMaster] Error: {error_msg}")
                        node.send_output("status", pa.array(["error"]))
                        node.send_output("error", pa.array([json.dumps(error_msg)]))

                elif event_id == "command":
                    # Handle control commands
                    try:
                        cmd_data = event["value"]
                        if hasattr(cmd_data, "to_pylist"):
                            command = cmd_data.to_pylist()[0]
                        else:
                            command = str(cmd_data)

                        command = command.strip().lower()

                        if command == "home":
                            result = driver.move_joints(driver.home_position, wait=True)
                        elif command == "safe":
                            result = driver.move_joints(driver.safe_position, wait=True)
                        elif command == "reset":
                            result = driver.move_joints(driver.safe_position, wait=True)
                        elif command == "get_state":
                            result = driver.get_current_state()
                            result["success"] = True
                        else:
                            result = {"success": False, "error": f"Unknown command: {command}"}

                        # Send result
                        if result["success"]:
                            node.send_output("status", pa.array(["completed"]))
                            if "joints" in result:
                                joints = np.array(result["joints"])
                                node.send_output("joint_positions", pa.array(joints, type=pa.float32()))
                        else:
                            node.send_output("status", pa.array(["error"]))
                            node.send_output("error", pa.array([json.dumps(result)]))

                    except Exception as e:
                        error_msg = {"success": False, "error": f"Command error: {str(e)}"}
                        node.send_output("status", pa.array(["error"]))
                        node.send_output("error", pa.array([json.dumps(error_msg)]))

            elif event_type == "STOP":
                print("[LM3-LMaster] Stopping...")
                break

    finally:
        driver.disconnect()
        print("[LM3-LMaster] Node shutdown complete")


if __name__ == "__main__":
    main()
