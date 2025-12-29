# Workflow Separation Guide

This document explains how the simulation and real robot workflows are separated in the GEN72 system.

## Overview

The system has **two completely independent workflows** that share common planning logic but use different robot control implementations:

1. **Simulation Workflow** - Uses MuJoCo physics simulator
2. **Real Robot Workflow** - Uses Realman SDK to control physical GEN72 robot

## Launch Commands

### Simulation Mode
```bash
./run_mujoco.bat
```
- Launches: `config/dataflow_gen72_mujoco.yml`
- Robot control: `dora-mujoco` (MuJoCo simulator)
- No physical robot required
- Safe for testing and development

### Real Robot Mode
```bash
./run_real_robot.bat
```
- Launches: `config/dataflow_gen72_real.yml`
- Robot control: `robot_control/gen72_robot_node.py` (Realman SDK)
- Requires physical robot at 192.168.1.18
- Safety warnings displayed before execution

## File Dependencies

### Simulation-Only Files
These files are **only used** when running `run_mujoco.bat`:

```
dora-mujoco/
├── dora_mujoco/
│   ├── __init__.py
│   ├── __main__.py
│   └── main.py                    # MuJoCo simulator node

utils/
├── visualize_scene.py             # MuJoCo visualization tool
└── create_scene_with_obstacles.py # URDF to MuJoCo converter
```

**Dependencies:**
- `mujoco` Python package
- `mujoco.viewer` for visualization
- MuJoCo model file: `config/GEN72_base.xml`

### Real Robot-Only Files
These files are **only used** when running `run_real_robot.bat`:

```
robot_control/
├── __init__.py
├── gen72_robot_node.py            # Real robot control node
├── rm_robot_interface.py          # Realman SDK Python interface
├── rm_ctypes_wrap.py              # Realman SDK C wrapper
└── api_c.dll                      # Realman SDK library (Windows)
```

**Dependencies:**
- Realman SDK (api_c.dll)
- Physical GEN72 robot connected to network
- Robot IP: 192.168.1.18:8080

### Shared Files
These files are used by **both workflows**:

```
workflow/
├── multi_view_capture_node.py     # Main workflow controller
└── motion_commander.py            # Motion command interface

ik_solver/
├── ik_op.py                       # TracIK inverse kinematics
└── advanced_ik_solver.py          # Advanced IK implementations

motion_planner/
├── planner_ompl_with_collision_op.py  # RRT-Connect planner
└── planning_scene_op.py           # Scene management

trajectory_execution/
└── trajectory_executor.py         # Trajectory interpolation

collision_detection/
├── collision_lib.py               # Core collision checking
├── collision_check_op.py          # Collision check operator
└── pointcloud_collision.py        # Point cloud integration

config/
├── robot_config.py                # GEN72 parameters
└── GEN72_base.xml                 # Robot model (MuJoCo format)
```

**Key Point:** These shared files are **robot-agnostic**. They work with joint positions and commands regardless of whether the robot is simulated or real.

## Dataflow Configuration Comparison

### Simulation Dataflow (`dataflow_gen72_mujoco.yml`)

```yaml
nodes:
  - id: mujoco_sim
    path: dora-mujoco                              # MuJoCo simulator
    inputs:
      tick: dora/timer/millis/10                   # 100 Hz simulation
      control_input: trajectory_executor/joint_commands
    outputs:
      - joint_positions
      - joint_velocities
    env:
      MODEL_NAME: "F:/2-DORA/.../GEN72_base.xml"

  - id: planning_scene
    inputs:
      robot_state: mujoco_sim/joint_positions      # From simulator

  - id: trajectory_executor
    inputs:
      joint_positions: mujoco_sim/joint_positions  # From simulator
      tick: dora/timer/millis/50                   # 20 Hz execution
```

### Real Robot Dataflow (`dataflow_gen72_real.yml`)

```yaml
nodes:
  - id: gen72_robot
    path: ../robot_control/gen72_robot_node.py     # Real robot control
    inputs:
      tick: dora/timer/millis/200                  # 5 Hz update
      control_input: trajectory_executor/joint_commands
    outputs:
      - joint_positions
      - joint_velocities

  - id: planning_scene
    inputs:
      robot_state: gen72_robot/joint_positions     # From real robot

  - id: trajectory_executor
    inputs:
      joint_positions: gen72_robot/joint_positions # From real robot
      tick: dora/timer/millis/200                  # 5 Hz execution
```

**Key Differences:**
1. **Robot control node**: `mujoco_sim` vs `gen72_robot`
2. **Update frequency**: 100 Hz (simulation) vs 5 Hz (real robot)
3. **Data source**: All other nodes receive `joint_positions` from different sources but process them identically

## How Separation Works

### 1. Dataflow-Level Separation
- Each workflow uses a different YAML configuration file
- The YAML files specify which robot control node to use
- All other nodes are identical in both configurations

### 2. No Runtime Conditionals
- **No `if simulation:` checks in code**
- **No environment variables to switch modes**
- Mode is determined purely by which dataflow YAML is loaded

### 3. Dependency Isolation
- Simulation workflow never imports `rm_robot_interface`
- Real robot workflow never imports `mujoco`
- Shared nodes import neither - they only work with numpy arrays

### 4. Interface Compatibility
Both robot control nodes provide the same interface:

**Inputs:**
- `tick`: Timer signal for updates
- `control_input`: Joint commands from trajectory executor

**Outputs:**
- `joint_positions`: Current joint angles (7D array)
- `joint_velocities`: Current joint velocities (7D array)

This interface compatibility allows all downstream nodes to work identically with both modes.

## Testing Independence

### Test Simulation Without Real Robot
```bash
# This should work even if:
# - Robot is not connected
# - Realman SDK is not installed
# - api_c.dll is missing
./run_mujoco.bat
```

### Test Real Robot Without MuJoCo
```bash
# This should work even if:
# - MuJoCo is not installed
# - GEN72_base.xml is missing
# - dora-mujoco package is not installed
./run_real_robot.bat
```

## Adding New Features

### To Add Simulation-Only Feature
1. Modify `dora-mujoco/dora_mujoco/main.py`
2. Update `config/dataflow_gen72_mujoco.yml` if needed
3. No changes to shared files

### To Add Real Robot-Only Feature
1. Modify `robot_control/gen72_robot_node.py`
2. Update `config/dataflow_gen72_real.yml` if needed
3. No changes to shared files

### To Add Feature for Both Workflows
1. Modify shared files (workflow, IK, planning, etc.)
2. Ensure changes work with generic joint position arrays
3. Test with both `run_mujoco.bat` and `run_real_robot.bat`

## Collision Detection Note

The collision detection system (`collision_detection/collision_check_op.py`) has a flag:

```python
USE_REALMAN_API = False  # Set to True to use Realman official collision detection
```

**Current behavior:**
- `False` (default): Uses built-in geometric collision checker (works for both modes)
- `True`: Uses Realman SDK collision API (only works in real robot mode)

**Recommendation:** Keep as `False` to maintain independence. The built-in checker works well for both workflows.

## Summary

✅ **Simulation and real robot workflows are fully separated**
- Different entry points: `run_mujoco.bat` vs `run_real_robot.bat`
- Different dataflow configs: `dataflow_gen72_mujoco.yml` vs `dataflow_gen72_real.yml`
- Different robot control nodes: `dora-mujoco` vs `gen72_robot_node.py`
- No shared dependencies between simulation-only and real-robot-only code
- Shared planning/IK/trajectory code is robot-agnostic

✅ **No modifications needed to existing code**
- Separation already exists through dataflow architecture
- No runtime conditionals or mode switches required
- Both workflows tested and working independently
