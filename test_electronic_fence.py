#!/usr/bin/env python3
"""
Test Electronic Fence Configuration
Tests the electronic fence setup for GEN72 robot
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_control.rm_robot_interface import *
from robot_control.electronic_fence_setup import setup_electronic_fence, get_fence_status, disable_electronic_fence


def main():
    print("=" * 60)
    print("GEN72 Electronic Fence Test")
    print("=" * 60)

    # Connect to robot
    print("\n[1] Connecting to robot...")
    thread_mode = rm_thread_mode_e(2)
    robot = RoboticArm(thread_mode)
    handle = robot.rm_create_robot_arm("192.168.1.19", 8080, 3)

    if handle.id == -1:
        print("ERROR: Failed to connect to GEN72 robot")
        print("Please check:")
        print("  - Robot is powered on")
        print("  - Network connection to 192.168.1.19")
        print("  - No other programs are connected to the robot")
        return

    print(f"SUCCESS: Connected to GEN72 (handle: {handle.id})")

    try:
        # Setup electronic fence
        print("\n[2] Setting up electronic fence...")
        success = setup_electronic_fence(robot)

        if not success:
            print("ERROR: Failed to setup electronic fence")
            return

        # Get and display fence status
        print("\n[3] Checking fence status...")
        status = get_fence_status(robot)
        print(f"Fence enabled: {status.get('enable', False)}")
        print(f"Mode: {status.get('mode', 'unknown')}")

        # Get fence configuration
        print("\n[4] Getting fence configuration...")
        result, config = robot.rm_get_electronic_fence_config()
        if result == 0:
            print("Fence configuration:")
            print(f"  Work frame: {config.get('work_frame', 'unknown')}")
            if 'cube' in config:
                cube = config['cube']
                print(f"  Cube boundaries:")
                print(f"    X: {cube.get('x_min', 0):.2f} to {cube.get('x_max', 0):.2f} m")
                print(f"    Y: {cube.get('y_min', 0):.2f} to {cube.get('y_max', 0):.2f} m")
                print(f"    Z: {cube.get('z_min', 0):.2f} to {cube.get('z_max', 0):.2f} m")
                print(f"\n  Robot CANNOT move behind Y = {cube.get('y_min', 0):.2f} m")
        else:
            print(f"Failed to get fence config: error code {result}")

        print("\n[5] Electronic fence is now active!")
        print("The robot will automatically reject any motion commands that")
        print("would cause the end-effector to move behind Y = -0.5m")

        # Wait for user input
        input("\nPress Enter to disable fence and exit...")

        # Disable fence
        print("\n[6] Disabling electronic fence...")
        disable_electronic_fence(robot)

    finally:
        # Disconnect
        print("\n[7] Disconnecting...")
        robot.rm_delete_robot_arm()
        print("Disconnected from robot")

    print("\n" + "=" * 60)
    print("Test completed successfully")
    print("=" * 60)


if __name__ == "__main__":
    main()
