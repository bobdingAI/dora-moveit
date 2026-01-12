#!/usr/bin/env python3
"""
Lebai LM3 Robot Configuration
6-DOF Robot Arm
Extracted from URDF: LM3-lebai/urdf-lebai_lm3/lm3_macro.xacro
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple

@dataclass
class JointConfig:
    """Single joint configuration"""
    name: str
    lower_limit: float  # rad
    upper_limit: float  # rad
    velocity_limit: float  # rad/s
    effort_limit: float  # Nm

class LM3Config:
    """Lebai LM3 6-DOF Robot Arm Configuration"""

    # Number of joints
    NUM_JOINTS = 6

    # DH Parameters from URDF
    D1 = 0.21583
    D4 = 0.12063
    D5 = 0.09833
    D6 = 0.08343
    A2 = -0.28
    A3 = -0.26

    # Joint limits (from URDF - all joints have unlimited range)
    JOINT_CONFIGS = [
        JointConfig("joint_1", -6.28, 6.28, 3.14, 100.0),
        JointConfig("joint_2", -6.28, 6.28, 3.14, 100.0),
        JointConfig("joint_3", -6.28, 6.28, 3.14, 100.0),
        JointConfig("joint_4", -6.28, 6.28, 3.14, 100.0),
        JointConfig("joint_5", -6.28, 6.28, 3.14, 5.5),
        JointConfig("joint_6", -6.28, 6.28, 3.14, 4.6),
    ]

    # Extract arrays for easy access
    JOINT_LOWER_LIMITS = np.array([j.lower_limit for j in JOINT_CONFIGS])
    JOINT_UPPER_LIMITS = np.array([j.upper_limit for j in JOINT_CONFIGS])
    JOINT_VELOCITY_LIMITS = np.array([j.velocity_limit for j in JOINT_CONFIGS])
    JOINT_EFFORT_LIMITS = np.array([j.effort_limit for j in JOINT_CONFIGS])

    # Link transforms from URDF (xyz, rpy)
    LINK_TRANSFORMS = [
        # joint_1: base_link -> link_1
        {"xyz": [0, 0, D1], "rpy": [0, 0, 0]},
        # joint_2: link_1 -> link_2
        {"xyz": [0, 0, 0], "rpy": [np.pi/2, 0, 0]},
        # joint_3: link_2 -> link_3
        {"xyz": [A2, 0, 0], "rpy": [0, 0, 0]},
        # joint_4: link_3 -> link_4
        {"xyz": [A3, 0, D4], "rpy": [0, 0, 0]},
        # joint_5: link_4 -> link_5
        {"xyz": [0, -D5, 0], "rpy": [np.pi/2, 0, 0]},
        # joint_6: link_5 -> link_6
        {"xyz": [0, D6, 0], "rpy": [-np.pi/2, 0, 0]},
    ]

    # Inertial parameters from URDF (mass, center of mass, inertia tensor)
    # Inertia format: [ixx, ixy, ixz, iyy, iyz, izz]
    LINK_INERTIAL = [
        # base_link (no inertial in URDF)
        {"mass": 0.0, "com": [0, 0, 0], "inertia": [0, 0, 0, 0, 0, 0]},
        # link_1
        {"mass": 2.147, "com": [0.0, -0.011, -0.015],
         "inertia": [0.0082391363, 0, 0, 0.0076087775, 0.00032383729, 0.0032573589]},
        # link_2
        {"mass": 1.972, "com": [-0.134, 0.0, 0.094],
         "inertia": [0.0017128976, 0.0, 0.000073751933, 0.033612233, 0.0, 0.03384798]},
        # link_3
        {"mass": 1.668, "com": [-0.102, 0.0, 0.019],
         "inertia": [0.0017092842, 0.0, -0.002750573, 0.022563964, 0.0, 0.022383877]},
        # link_4
        {"mass": 0.969, "com": [0.0, 0.011, -0.039],
         "inertia": [0.0018744072, 0.0, 0.0, 0.0017809517, -0.00040039676, 0.00079617326]},
        # link_5
        {"mass": 0.969, "com": [0.0, -0.011, -0.039],
         "inertia": [0.0018744072, 0.0, 0.0, 0.0017809517, 0.00040039676, 0.00079617326]},
        # link_6
        {"mass": 0.584, "com": [0, 0, -0.049],
         "inertia": [0.00050434988, 0.0, 0.0, 0.0005155908, 0.0, 0.0004119313]},
    ]

    # Collision geometry (simplified spheres for each link)
    # Radii adjusted to reduce false collision detection
    COLLISION_GEOMETRY = [
        ("sphere", [0.045]),  # base_link: 6cm → 4.5cm (-25%)
        ("sphere", [0.045]),  # link_1: 5.5cm → 4.5cm
        ("sphere", [0.040]),  # link_2: 5cm → 4cm
        ("sphere", [0.040]),  # link_3: 5cm → 4cm (-20%)
        ("sphere", [0.035]),  # link_4: 4cm → 3.5cm
        ("sphere", [0.035]),  # link_5: 4cm → 3.5cm
        ("sphere", [0.030]),  # link_6: 3.5cm → 3cm
    ]

    # Safety parameters
    COLLISION_MARGIN = 0.015  # 1.5cm safety margin
    MAX_ACCELERATION = np.array([5.0, 5.0, 5.0, 5.0, 8.0, 8.0])  # rad/s^2

    # Default home configuration (safe extended pose)
    # Joint angles: J1=-118°, J2=-89°, J3=-93°, J4=-87°, J5=88°, J6=-79°
    HOME_CONFIG = np.array([-2.0595, -1.5533, -1.6232, -1.5184, 1.5359, -1.3788])

    # Safe initial configuration (extended, natural pose without self-collision)
    # Joint angles: J1=-118°, J2=-89°, J3=-93°, J4=-87°, J5=88°, J6=-79°
    SAFE_CONFIG = np.array([-2.0595, -1.5533, -1.6232, -1.5184, 1.5359, -1.3788])

    @staticmethod
    def get_joint_limits() -> Tuple[np.ndarray, np.ndarray]:
        """Get joint position limits"""
        return LM3Config.JOINT_LOWER_LIMITS, LM3Config.JOINT_UPPER_LIMITS

    @staticmethod
    def get_velocity_limits() -> np.ndarray:
        """Get joint velocity limits"""
        return LM3Config.JOINT_VELOCITY_LIMITS

    @staticmethod
    def is_config_valid(q: np.ndarray) -> bool:
        """Check if configuration is within joint limits"""
        return np.all(q >= LM3Config.JOINT_LOWER_LIMITS) and \
               np.all(q <= LM3Config.JOINT_UPPER_LIMITS)

    @staticmethod
    def clip_to_limits(q: np.ndarray) -> np.ndarray:
        """Clip configuration to joint limits"""
        return np.clip(q, LM3Config.JOINT_LOWER_LIMITS, LM3Config.JOINT_UPPER_LIMITS)
