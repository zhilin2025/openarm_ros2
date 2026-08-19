# OpenArmX Deploy (ROS2 C++)

ROS2 C++ 节点，使用 LibTorch 将训练好的视觉抓取策略部署到真实机械臂。

## 目录结构

```
openarmx_deploy/
├── CMakeLists.txt              # 构建配置
├── package.xml                 # ROS2 包元信息
├── include/openarmx_deploy/
│   ├── grasp_state_machine.h   # 六阶段抓取状态机
│   └── observation_builder.h   # 29 维观测构造器
├── src/
│   ├── deploy_node.cpp         # ROS2 节点主程序
│   ├── grasp_state_machine.cpp
│   └── observation_builder.cpp
├── scripts/
│   └── export_model.py         # Python → TorchScript 模型导出
└── launch/
    └── deploy.launch.py        # ROS2 Launch 文件
```

## 构建步骤

### 1. 前置依赖
```bash
# 安装 ROS2 Humble (或 Iron/Jazzy)
# 安装 LibTorch: https://pytorch.org/get-started/locally/
#   下载解压后设置环境变量:
export Torch_DIR=/path/to/libtorch/share/cmake/Torch

# 安装 MoveIt2 (回位避障需要):
sudo apt install ros-$ROS_DISTRO-moveit
```

### 2. 导出模型 (Python → TorchScript)
```bash
cd project/openarmx_deploy/scripts
python3 export_model.py \
    --checkpoint ../checkpoints/jgzh_sim2real.pth \
    --output ../checkpoints/jgzh_sim2real.pt
```

### 3. 构建 ROS2 包
```bash
cd ~/ros2_ws/src
ln -s /path/to/project/openarmx_deploy .
cd ~/ros2_ws
colcon build --packages-select openarmx_deploy --cmake-args -DTorch_DIR=$Torch_DIR
source install/setup.bash
```

### 4. 运行
```bash
# 先启动 MoveIt2 (move_group + 机器人控制器)
ros2 launch <你的机器人>_moveit_config move_group.launch.py

# 再启动部署节点
ros2 launch openarmx_deploy deploy.launch.py \
    robot:=JGZH \
    checkpoint:=checkpoints/jgzh_sim2real.pt \
    move_group_name:=arm      # 需匹配你的 MoveIt 规划组名

# 触发抓取
ros2 service call /openarmx_grasp std_srvs/srv/Trigger

# 复位 (自动回初始位姿, 含 MoveIt2 避障规划)
ros2 service call /openarmx_reset std_srvs/srv/Trigger
```

### 5. 回位避障说明

复位时 `moveToHome()` 优先通过 **MoveIt2** 规划回初始位姿：

1. `MoveGroupInterface` 以 `move_group_name` 规划组创建
2. `setStartStateToCurrentState()` — 从当前真实关节状态开始
3. `setJointValueTarget(home_qpos_)` — 关节空间目标
4. `plan()` — OMPL 基于 planning scene 避障规划
5. `execute()` — 执行规划轨迹 (速度缩放 30%)
6. 失败时自动**降级**为无避障的关节插值，并打印警告

**避障生效的前提**：
- MoveIt 的 planning scene 中必须有障碍物（感知节点发布 collision object 到 `/planning_scene`，或使用 planning scene monitor 自动感知）
- `move_group` 节点已启动，规划组名与 `move_group_name` 参数一致
- 节点需要 **MultiThreadedExecutor**（已配置）避免 action 回调与服务回调死锁

## 部署前必须调整

| 项目 | 说明 |
|------|------|
| **关节名称** | `kRobotConfigs` 中的关节名需匹配实际 `/joint_states` |
| **IK** | `solveIK()` 当前是占位，需接入 MoveIt2 IK 服务 |
| **TCP 位姿** | `getCurrentTCP()` 当前是近似，需通过 TF/FK 计算 |
| **物体位姿** | 需要感知系统发布 `/object_pose` |
| **MoveIt 规划组** | `move_group_name` 参数需匹配你的 MoveIt 配置 |

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `home_pose` | `""` | 固定初始关节角 (逗号分隔 7 个)；空=启动时记录 |
| `home_tolerance` | `0.02` | 回位到位容差 (rad) |
| `home_timeout` | `30.0` | 回位超时 (s) |
| `use_moveit` | `true` | 回位是否使用 MoveIt2 避障规划 |
| `move_group_name` | `arm` | MoveIt2 规划组名 |
| `moveit_velocity_scale` | `0.3` | 回位速度缩放 (0~1) |

## 话题 & 服务

| 名称 | 类型 | 方向 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 订阅 |
| `/object_pose` | `geometry_msgs/PoseStamped` | 订阅 |
| `/arm_joint_command` | `trajectory_msgs/JointTrajectory` | 发布 |
| `/gripper_command` | `std_msgs/Float64MultiArray` | 发布 |
| `/openarmx_grasp` | `std_srvs/Trigger` | 服务 |
| `/openarmx_reset` | `std_srvs/Trigger` | 服务 |
