# LM3 机械臂适配说明

本项目已成功将GEN72 7轴机械臂替换为乐白LM3 6轴机械臂。

## 已完成的修改

### 1. 配置文件
- **[config/lm3_config.py](config/lm3_config.py)** - LM3机械臂参数配置
  - 6个关节的限位、速度、力矩参数
  - DH参数（从URDF提取）
  - 碰撞几何体定义
  - HOME位置配置

### 2. IK求解器
- **[ik_solver/ik_op_lm3.py](ik_solver/ik_op_lm3.py)** - LM3专用IK求解器节点
- **[ik_solver/advanced_ik_solver_lm3.py](ik_solver/advanced_ik_solver_lm3.py)** - LM3的TracIK和优化求解器
  - 支持6轴机械臂
  - Jacobian方法
  - BFGS优化
  - 多起点随机搜索

### 3. 实体机械臂控制
- **[robot_control/lm3_robot_node.py](robot_control/lm3_robot_node.py)** - LM3实体机械臂控制节点
  - 使用lebai_sdk连接机械臂
  - 支持关节位置读取和控制
  - Mock模式（无SDK时可测试）

### 4. MuJoCo仿真
- **[config/LM3_base.xml](config/LM3_base.xml)** - LM3的MuJoCo模型文件
  - 6个关节的运动学定义
  - 简化几何体（圆柱、胶囊体）
  - 惯性参数（从URDF提取）

### 5. Dataflow配置
- **[config/dataflow_lm3_mujoco.yml](config/dataflow_lm3_mujoco.yml)** - 仿真模式配置
- **[config/dataflow_lm3_real.yml](config/dataflow_lm3_real.yml)** - 实体机械臂模式配置

### 6. 启动脚本
- **[run_lm3_mujoco.bat](run_lm3_mujoco.bat)** - 启动MuJoCo仿真
- **[run_lm3_real.bat](run_lm3_real.bat)** - 启动实体机械臂控制

## 快速开始

### 前置条件

```bash
# 安装依赖
pip install -r requirements.txt

# 安装Dora
pip install dora-rs

# 安装MuJoCo（仿真需要）
pip install mujoco

# 安装乐白SDK（实体机械臂需要）
pip install lebai-sdk
```

### 运行仿真

```bash
# Windows
run_lm3_mujoco.bat

# Linux/Mac
dora up
dora start config/dataflow_lm3_mujoco.yml --attach
```

### 运行实体机械臂

**重要：确保机械臂已上电，工作空间清空，急停按钮可触及**

```bash
# 设置机械臂IP（可选，默认192.168.1.200）
set LEBAI_IP=192.168.1.200

# Windows
run_lm3_real.bat

# Linux/Mac
export LEBAI_IP=192.168.1.200
dora up
dora start config/dataflow_lm3_real.yml --attach
```

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    LM3 System Architecture                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   multi_view_capture_node.py (Workflow Controller)             │
│       │                                                          │
│       ├──► ik_op_lm3.py (TracIK Solver - 6 DOF)                │
│       │         └──► Cartesian pose → 6 Joint angles           │
│       │                                                          │
│       ├──► planner_ompl_with_collision_op.py (RRT-Connect)     │
│       │         ├──► Collision-free path planning              │
│       │         └──► Point cloud + geometric collision         │
│       │                                                          │
│       ├──► trajectory_executor.py (Interpolation)              │
│       │         └──► Smooth waypoint interpolation             │
│       │                                                          │
│       └──► Robot Control Node (Mode-Specific)                   │
│             ├──► Simulation: dora-mujoco (MuJoCo Simulator)    │
│             └──► Real Robot: lm3_robot_node.py (Lebai SDK)     │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## LM3参数

### 关节限位
- 所有关节：-6.28 ~ 6.28 rad（几乎无限制）
- 速度限制：3.14 rad/s
- 力矩限制：
  - Joint 1-4: 100 Nm
  - Joint 5: 5.5 Nm
  - Joint 6: 4.6 Nm

### DH参数
- d1 = 0.21583 m
- d4 = 0.12063 m
- d5 = 0.09833 m
- d6 = 0.08343 m
- a2 = -0.28 m
- a3 = -0.26 m

### HOME位置
```python
HOME_CONFIG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 所有关节归零
```

## 与GEN72的主要区别

| 特性 | GEN72 | LM3 |
|------|-------|-----|
| 自由度 | 7轴 | 6轴 |
| 关节限位 | 各不相同 | 几乎无限制（±6.28 rad） |
| 控制SDK | Realman SDK | Lebai SDK |
| 工作空间 | 较大 | 中等 |
| 末端负载 | 较高 | 中等 |

## 注意事项

1. **IK求解**：6轴机械臂的IK解可能不唯一，系统会选择最接近当前位置的解
2. **关节限位**：虽然LM3关节几乎无限制，但实际使用时建议设置合理的软限位
3. **碰撞检测**：MuJoCo模型使用简化几何体，实际碰撞检测可能需要更精确的模型
4. **运动抖动**：如遇到运动抖动，可调整trajectory_executor.py中的插值速度参数

## 文件对照表

| GEN72文件 | LM3文件 | 说明 |
|-----------|---------|------|
| config/robot_config.py | config/lm3_config.py | 机械臂参数 |
| ik_solver/ik_op.py | ik_solver/ik_op_lm3.py | IK求解器 |
| robot_control/gen72_robot_node.py | robot_control/lm3_robot_node.py | 实体控制 |
| config/GEN72_base.xml | config/LM3_base.xml | MuJoCo模型 |
| config/dataflow_gen72_mujoco.yml | config/dataflow_lm3_mujoco.yml | 仿真配置 |
| config/dataflow_gen72_real.yml | config/dataflow_lm3_real.yml | 实体配置 |
| run_mujoco.bat | run_lm3_mujoco.bat | 仿真启动 |
| run_real_robot.bat | run_lm3_real.bat | 实体启动 |

## 故障排查

### 仿真问题

**问题**：MuJoCo窗口无法打开
```bash
# 检查模型文件路径
# 确保config/LM3_base.xml存在
```

**问题**：关节运动异常
```bash
# 检查关节限位配置
# 查看config/lm3_config.py中的JOINT_LIMITS
```

### 实体机械臂问题

**问题**：无法连接机械臂
```bash
# 检查网络连接
ping 192.168.1.200

# 检查SDK安装
pip list | grep lebai

# 检查IP配置
echo %LEBAI_IP%  # Windows
echo $LEBAI_IP   # Linux/Mac
```

**问题**：运动抖动严重
```bash
# 降低控制频率
# 修改dataflow_lm3_real.yml中的tick频率
# 从150ms增加到200ms或更高
```

## 下一步优化

1. **精确模型**：使用LM3的STL文件替换简化几何体
2. **轨迹优化**：实现五次多项式插值减少抖动
3. **力控制**：集成力传感器实现柔顺控制
4. **视觉集成**：接入相机进行视觉伺服

## 参考资料

- [乐白官方SDK](https://github.com/lebai-robotics/lebai-sdk)
- [乐白开发手册](http://docs.lebai.ltd/lmaster/api-doc/)
- [Dora-rs文档](https://github.com/dora-rs/dora)
- [MuJoCo文档](https://mujoco.readthedocs.io/)
