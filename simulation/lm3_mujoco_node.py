#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
"""
LM3 MuJoCo Simulation Node (Improved)
Applies best practices from Lebai project:
- Structured return values
- Environment variable configuration
- Status reporting
- Command channel
- Input validation
"""

import json
import numpy as np
import pyarrow as pa
from dora import Node
from config.lm3_config import LM3Config

try:
    import mujoco
    import mujoco.viewer
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False
    print("[MuJoCo-Sim] WARNING: mujoco not available")


class LM3MuJoCoSimulator:
    """LM3 MuJoCo simulator with improved error handling"""

    def __init__(self):
        # Environment variable configuration
        self.model_path = os.getenv("MUJOCO_MODEL_PATH",
                                    "config/LM3_base.xml")
        self.render_mode = os.getenv("MUJOCO_RENDER", "true").lower() == "true"
        self.sim_dt = float(os.getenv("MUJOCO_DT", "0.01"))  # 10ms = 100Hz

        self.model = None
        self.data = None
        self.viewer = None
        self.current_joints = LM3Config.SAFE_CONFIG.copy()
        self.target_joints = LM3Config.SAFE_CONFIG.copy()
        self.is_initialized = False

        # Initialize simulation
        result = self.initialize()
        if not result["success"]:
            print(f"[MuJoCo-Sim] Initialization failed: {result['error']}")

    def initialize(self) -> dict:
        """Initialize MuJoCo simulation

        Returns:
            dict: {"success": bool, "message": str} or {"success": bool, "error": str}
        """
        if not MUJOCO_AVAILABLE:
            return {"success": False, "error": "MuJoCo not available"}

        try:
            # Load model
            if not os.path.exists(self.model_path):
                return {"success": False, "error": f"Model file not found: {self.model_path}"}

            self.model = mujoco.MjModel.from_xml_path(self.model_path)
            self.data = mujoco.MjData(self.model)

            # Validate model
            if self.model.nu != 6:
                return {"success": False, "error": f"Expected 6 actuators, got {self.model.nu}"}

            # Set initial position
            self.data.qpos[:6] = LM3Config.SAFE_CONFIG
            mujoco.mj_forward(self.model, self.data)

            # Initialize viewer if rendering
            if self.render_mode:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

            self.current_joints = self.data.qpos[:6].copy()
            self.target_joints = self.current_joints.copy()
            self.is_initialized = True

            return {
                "success": True,
                "message": f"MuJoCo initialized: {self.model_path}"
            }

        except Exception as e:
            return {"success": False, "error": str(e)}

    def step(self) -> dict:
        """Step simulation forward

        Returns:
            dict: {"success": bool, "joints": list, "velocities": list} or {"success": bool, "error": str}
        """
        if not self.is_initialized:
            return {"success": False, "error": "Simulator not initialized"}

        try:
            # Set control input
            self.data.ctrl[:] = self.target_joints

            # Step simulation
            mujoco.mj_step(self.model, self.data)

            # Update viewer
            if self.render_mode and self.viewer is not None:
                self.viewer.sync()

            # Read state
            self.current_joints = self.data.qpos[:6].copy()
            velocities = self.data.qvel[:6].copy()

            return {
                "success": True,
                "joints": self.current_joints.tolist(),
                "velocities": velocities.tolist()
            }

        except Exception as e:
            return {"success": False, "error": str(e)}

    def set_target(self, target_joints: np.ndarray) -> dict:
        """Set target joint positions

        Args:
            target_joints: Target joint angles (6 values)

        Returns:
            dict: {"success": bool, "message": str} or {"success": bool, "error": str}
        """
        # Input validation
        if len(target_joints) != 6:
            return {"success": False, "error": f"Expected 6 joint values, got {len(target_joints)}"}

        # Range validation
        for i, q in enumerate(target_joints):
            if not (-6.28 <= q <= 6.28):
                return {"success": False, "error": f"Joint {i} out of range: {q}"}

        self.target_joints = target_joints.copy()
        return {"success": True, "message": "Target set"}

    def get_state(self) -> dict:
        """Get current simulator state

        Returns:
            dict: State information
        """
        return {
            "initialized": self.is_initialized,
            "render_mode": self.render_mode,
            "model_path": self.model_path,
            "current_joints": self.current_joints.tolist(),
            "target_joints": self.target_joints.tolist(),
            "sim_dt": self.sim_dt
        }

    def reset(self) -> dict:
        """Reset simulation to safe configuration

        Returns:
            dict: {"success": bool, "message": str} or {"success": bool, "error": str}
        """
        if not self.is_initialized:
            return {"success": False, "error": "Simulator not initialized"}

        try:
            self.data.qpos[:6] = LM3Config.SAFE_CONFIG
            self.data.qvel[:] = 0
            mujoco.mj_forward(self.model, self.data)

            self.current_joints = self.data.qpos[:6].copy()
            self.target_joints = self.current_joints.copy()

            return {"success": True, "message": "Simulation reset"}

        except Exception as e:
            return {"success": False, "error": str(e)}

    def close(self):
        """Close simulator and cleanup resources"""
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
        self.is_initialized = False


def main():
    """Main entry point for MuJoCo simulation node"""
    print("=== LM3 MuJoCo Simulation Node (Improved) ===")

    node = Node()
    sim = LM3MuJoCoSimulator()

    if not sim.is_initialized:
        print("[MuJoCo-Sim] Failed to initialize, exiting")
        return

    print("[MuJoCo-Sim] Node started, simulation running...")

    # Send initial state
    node.send_output("joint_positions", pa.array(sim.current_joints, type=pa.float32()))
    node.send_output("status", pa.array(["idle"]))

    try:
        for event in node:
            event_type = event["type"]

            if event_type == "INPUT":
                input_id = event["id"]

                if input_id == "tick":
                    # Step simulation
                    result = sim.step()

                    if result["success"]:
                        # Send joint state
                        joints = np.array(result["joints"])
                        velocities = np.array(result["velocities"])

                        node.send_output("joint_positions", pa.array(joints, type=pa.float32()))
                        node.send_output("joint_velocities", pa.array(velocities, type=pa.float32()))
                        node.send_output("status", pa.array(["running"]))
                    else:
                        node.send_output("status", pa.array(["error"]))
                        node.send_output("error", pa.array([json.dumps(result)]))

                elif input_id == "control_input":
                    # Receive control command
                    data = event["value"]

                    try:
                        # Parse input (support both array and JSON)
                        if hasattr(data, "to_pylist"):
                            data_list = data.to_pylist()
                            if data_list:
                                data_str = data_list[0]
                                if isinstance(data_str, str):
                                    cmd = json.loads(data_str)
                                else:
                                    cmd = data_str
                            else:
                                continue
                        else:
                            cmd = data.to_numpy()

                        # Extract joints
                        if isinstance(cmd, dict):
                            target = np.array(cmd.get("joints", cmd.get("target", [])))
                        else:
                            target = np.array(cmd)

                        # Set target
                        result = sim.set_target(target)

                        if result["success"]:
                            node.send_output("status", pa.array(["moving"]))
                        else:
                            node.send_output("status", pa.array(["error"]))
                            node.send_output("error", pa.array([json.dumps(result)]))

                    except Exception as e:
                        error_msg = {"success": False, "error": f"Parse error: {str(e)}"}
                        node.send_output("status", pa.array(["error"]))
                        node.send_output("error", pa.array([json.dumps(error_msg)]))

                elif input_id == "command":
                    # Handle control commands
                    try:
                        cmd_data = event["value"]
                        if hasattr(cmd_data, "to_pylist"):
                            command = cmd_data.to_pylist()[0]
                        else:
                            command = str(cmd_data)

                        if command == "reset":
                            result = sim.reset()
                        elif command == "get_state":
                            result = sim.get_state()
                            result["success"] = True
                        elif command == "home":
                            result = sim.set_target(LM3Config.HOME_CONFIG)
                        elif command == "safe":
                            result = sim.set_target(LM3Config.SAFE_CONFIG)
                        else:
                            result = {"success": False, "error": f"Unknown command: {command}"}

                        # Send result
                        if result["success"]:
                            node.send_output("status", pa.array(["completed"]))
                            if "joints" in result:
                                node.send_output("joint_positions",
                                               pa.array(result["joints"], type=pa.float32()))
                        else:
                            node.send_output("status", pa.array(["error"]))
                            node.send_output("error", pa.array([json.dumps(result)]))

                    except Exception as e:
                        error_msg = {"success": False, "error": f"Command error: {str(e)}"}
                        node.send_output("status", pa.array(["error"]))
                        node.send_output("error", pa.array([json.dumps(error_msg)]))

            elif event_type == "STOP":
                print("[MuJoCo-Sim] Stopping...")
                break

    finally:
        sim.close()
        print("[MuJoCo-Sim] Simulation closed")


if __name__ == "__main__":
    main()
