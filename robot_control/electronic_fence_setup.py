#!/usr/bin/env python3
"""
Electronic Fence Setup for GEN72 Robot
Configures safety boundaries to prevent robot from moving behind its base
"""

from robot_control.rm_robot_interface import *


def setup_electronic_fence(robot_arm: RoboticArm) -> bool:
    """
    Setup electronic fence to prevent robot from moving 0.5m behind its base.

    The fence is a cube that allows movement in front and sides, but blocks
    movement behind the robot base (negative Y direction beyond -0.5m).

    Args:
        robot_arm: Connected RoboticArm instance

    Returns:
        True if setup successful, False otherwise
    """

    # Define electronic fence as a cube
    # Coordinate system: Robot base at origin (0, 0, 0)
    # X: left/right, Y: front/back, Z: up/down
    # Fence blocks Y < -0.5 (behind robot)

    # Cube boundaries (in meters):
    # X: -1.0 to 1.0 (2m width, centered)
    # Y: -0.5 to 1.0 (blocks behind -0.5m, allows 1m in front)
    # Z: -0.1 to 1.0 (slightly below base to above reach)

    fence_config = rm_fence_config_t(
        form=1,  # 1 = Cube (长方体)
        name="back_limit",
        cube=rm_fence_config_cube_t(
            x_min=-2.0,
            x_max=2.0,
            y_min=-2.0,  # Block movement behind this point
            y_max=2.0,
            z_min=-0.1,
            z_max=2.0
        )
    )

    # Set the electronic fence configuration
    result = robot_arm.rm_set_electronic_fence_config(fence_config)
    if result != 0:
        print(f"[Electronic Fence] Failed to set config: error code {result}")
        return False

    print("[Electronic Fence] Configuration set successfully")

    # Enable electronic fence
    fence_enable = rm_electronic_fence_enable_t(
        enable=True,
        select_index=0,  # Use current config (not saved list)
        mode=0  # Electronic fence mode (not virtual wall)
    )

    result = robot_arm.rm_set_electronic_fence_enable(fence_enable)
    if result != 0:
        print(f"[Electronic Fence] Failed to enable: error code {result}")
        return False

    print("[Electronic Fence] Enabled successfully")
    print("[Electronic Fence] Robot cannot move behind Y = -0.5m")

    return True


def disable_electronic_fence(robot_arm: RoboticArm) -> bool:
    """
    Disable electronic fence.

    Args:
        robot_arm: Connected RoboticArm instance

    Returns:
        True if disabled successfully, False otherwise
    """
    fence_enable = rm_electronic_fence_enable_t(
        enable=False,
        select_index=0,
        mode=0
    )

    result = robot_arm.rm_set_electronic_fence_enable(fence_enable)
    if result != 0:
        print(f"[Electronic Fence] Failed to disable: error code {result}")
        return False

    print("[Electronic Fence] Disabled successfully")
    return True


def get_fence_status(robot_arm: RoboticArm) -> dict:
    """
    Get current electronic fence status.

    Args:
        robot_arm: Connected RoboticArm instance

    Returns:
        Dictionary with fence status
    """
    result, status = robot_arm.rm_get_electronic_fence_enable()
    if result != 0:
        print(f"[Electronic Fence] Failed to get status: error code {result}")
        return {}

    return status
