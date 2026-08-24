# OpenArmX Deploy (ROS2 C++)

ROS2 C++ 节点，使用 LibTorch 将训练好的视觉抓取策略部署到真实 OpenArmX 机械臂。

## 目录结构

```
openarmx_deploy/
├── CMakeLists.txt
├── package.xml
├── include/openarmx_deploy/
│   ├── arm_kinematics.h        # 7-DOF 正/逆运动学 (进程内 FK + 数值 DLS IK)
│   ├── grasp_state_machine.h   # 六阶段抓取状态机
│   └── observation_builder.h   # 29 维观测构造器
├── src/
│   ├── deploy_node.cpp         # ROS2 节点主程序
│   ├── arm_kinematics.cpp
│   ├── grasp_state_machine.cpp
│   └── observation_builder.cpp
├── scripts/
│   └── export_model.py         # Python → TorchScript 模型导出
├── models/
│   ├── jgzh_sim2real.pth       # 训练 checkpoint
│   └── jgzh_sim2real.pt        # 导出后的 TorchScript (供 C++ 加载)
└── launch/
    └── deploy.launch.py
```

## 数据流

```
/yolo_detection/object_poses (PoseArray)   ──┐
                                             ├─► 观测构造 (29 维) ─► Actor 策略 (LibTorch)
/joint_states (右臂关节角)                 ──┘          │
                                                        ▼
                            目标 TCP (expert + 策略修正) ─► 数值 IK (DLS)
                                                        ▼
    /right_joint_trajectory_controller/follow_joint_trajectory (Action)
    /right_gripper_controller/gripper_cmd                  (Action)
```

## 构建步骤

### 1. 前置依赖

- ROS2 Humble
- LibTorch CPU 版（推理为纯 CPU，用 CPU 版最干净、无 CUDA 运行时依赖）

> 注意：`yolo_venv` 里的 torch 是 **CUDA 版**（`2.9.1+cu126`），其 CMake 配置硬编码要求 CUDA
> 工具链（`nvcc`），本机未安装会直接报错，因此 C++ 构建请使用独立的 CPU 版 LibTorch。

```bash
# 下载并解压 CPU 版 LibTorch 2.9.1（与导出模型的 torch 版本一致）
cd ~
curl -L -o libtorch-cpu-2.9.1.zip \
  "https://download.pytorch.org/libtorch/cpu/libtorch-shared-with-deps-2.9.1%2Bcpu.zip"
unzip -q libtorch-cpu-2.9.1.zip -d libtorch-cpu

# 构建时指向它
export Torch_DIR=$HOME/libtorch-cpu/libtorch/share/cmake/Torch
```

> CMake 已把 `$TORCH_INSTALL_PREFIX/lib` 写入可执行文件的 rpath，运行时无需再设 `LD_LIBRARY_PATH`。

### 2. 导出模型 (Python → TorchScript)

```bash
cd src/openarm_ros2/openarm_rl_deploy
<yolo_venv>/bin/python3 scripts/export_model.py \
    --checkpoint models/jgzh_sim2real.pth \
    --output models/jgzh_sim2real.pt
```

### 3. 构建 ROS2 包

```bash
cd ~/openarm_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select openarmx_deploy \
    --cmake-args -DTorch_DIR=$Torch_DIR
source install/setup.bash
```

## 运行

真实机械臂先启动控制系统（`openarm.bimanual.launch.py` + GUI
`openarm_visual_controller.py`），再启动部署节点：

```bash
ros2 launch openarmx_deploy deploy.launch.py \
    arm_side:=right \
    checkpoint:=models/jgzh_sim2real.pt

# 触发抓取
ros2 service call /openarmx_grasp std_srvs/srv/Trigger

# 按 Gazebo v11 workcell 中水瓶中心发布假检测数据
ros2 topic pub /yolo_detection/object_poses geometry_msgs/msg/PoseArray \
"{header: {frame_id: openarm_body_link0}, poses: [{position: {x: -0.24, y: -0.44, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}]}" \
-r 5

# 复位 (回初始位姿)
ros2 service call /openarmx_reset std_srvs/srv/Trigger
```

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `arm_side` | `right` | 控制哪只手臂 (`left`/`right`) |
| `checkpoint` | `models/jgzh_sim2real.pt` | TorchScript 模型路径 (相对路径按包 share 目录解析) |
| `object_pose_topic` | `/yolo_detection/object_poses` | 物体位姿话题 (PoseArray) |
| `base_frame` | `openarm_body_link0` | FK/IK 使用的机械臂根坐标系；检测结果通过 TF 转到此坐标系 |
| `object_timeout` | `1.0` | 超过此时间未更新的目标不允许触发抓取 (s) |
| `table_z` | `0.80` | 桌面高度 (臂基座系)，用于状态机 above-table 裁剪 |
| `control_rate` | `30.0` | 策略观测与推理频率 (Hz)，不是单段轨迹时长 |
| `max_joint_velocity` | `0.50` | 抓取轨迹段的近似峰值关节速度上限 (rad/s) |
| `min_arm_goal_duration` | `0.25` | 抓取轨迹段最短执行时间 (s) |
| `gripper_goal_epsilon` | `0.0005` | 夹爪目标至少变化此距离才发送新 Action (m) |
| `action_pos_scale` | `0.03` | TCP 位置修正量缩放 (m) |
| `action_gripper_scale` | `0.005` | 夹爪修正量缩放 |
| `home_pose` | `""` | 固定初始关节角 (逗号分隔 7 个)；空=启动时记录 |

## 话题 / Action / 服务

| 名称 | 类型 | 方向 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 订阅 |
| `/yolo_detection/object_poses` | `geometry_msgs/PoseArray` | 订阅 |
| `/right_joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | Action 客户端 |
| `/right_gripper_controller/gripper_cmd` | `control_msgs/GripperCommand` | Action 客户端 |
| `/openarmx_grasp` | `std_srvs/Trigger` | 服务 |
| `/openarmx_reset` | `std_srvs/Trigger` | 服务 |

## Gazebo 联调

仿真相机话题、深度单位和桌面高度不同于真实相机默认配置，应显式传参：

```bash
# 终端 1
source /opt/ros/humble/setup.bash
source /home/zl/openarm_ws/install/setup.bash
ros2 launch openarm_gazebo_sim sim.launch.py rviz:=true

# 终端 2（必须先 source ROS，脚本会再加载 yolo_venv 中的 ultralytics）
source /opt/ros/humble/setup.bash
python3 /home/zl/openarm_ws/src/robstride_driver/scripts/yolo_bottle_detector.py \
  --ros-args \
  -p rgb_topic:=/camera/color/image_raw \
  -p depth_topic:=/camera/depth/image_rect_raw \
  -p camera_info_topic:=/camera/color/camera_info \
  -p show_local_window:=false

# 终端 3；仿真桌面顶面在 openarm_body_link0 中约为 z=0.105 m
source /opt/ros/humble/setup.bash
source /home/zl/openarm_ws/install/setup.bash
ros2 launch openarmx_deploy deploy.launch.py \
  use_sim_time:=true table_z:=0.105 arm_side:=right \
  max_joint_velocity:=0.50 min_arm_goal_duration:=0.25
```

`control_rate` 仍保持 30 Hz 运行策略闭环，但臂控制器同一时刻只执行一个完整轨迹段。
每段轨迹包含当前关节位置和 IK 目标两个零速度端点，执行时间根据最大关节位移与
`max_joint_velocity` 计算；上一段结束后才发送下一次修正，避免连续抢占 33 ms
单点轨迹造成视觉上的瞬移。

工作站模型中的水瓶中心在 world 坐标为 `(-0.14, -0.44, 0.59)`，而
`openarm_body_link0` 原点在 world 的 `z=0.38`，因此假检测坐标应使用
`(-0.14, -0.44, 0.21)`。该坐标是物体中心；状态机会先到其上方 0.15 m，
再下降到抓取高度。

触发前至少确认：

```bash
ros2 control list_controllers
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_rect_raw
ros2 topic echo /yolo_detection/object_poses --once
ros2 run tf2_ros tf2_echo openarm_body_link0 camera_depth_optical_frame
```

任一检查失败时不要触发 `/openarmx_grasp`。

Gazebo 相机及 image_transport 可能在 RViz/rqt/YOLO 建立订阅后才开始完整发布，
启动初期也可能因模型加载而延迟。`/camera/camera/.../compressed*`、
`compressedDepth` 和 `theora` 是压缩传输派生话题；YOLO 应继续使用上面列出的
RGB、深度和 CameraInfo 原始话题。

## 已知限制

- **相机外参**：部署节点已使用 TF 转换坐标，但真实相机仍必须完成可靠的手眼/外参标定；有 TF 不等于标定准确。
- **末端定义**：当前 FK/IK 使用 `openarm_*_link7` 原点，URDF 中实际夹持区域还沿工具方向偏移约 0.119 m。
- **姿态约束**：状态机生成了俯视抓取姿态，但当前 DLS IK 只求位置，尚未约束末端姿态。
- **成功判定**：`Grasp sequence completed` 只表示状态机走完，没有接触、夹爪堵转或物体抬升传感器判定。
- **策略来源**：文件名为 `jgzh_sim2real`，仓库中没有训练环境/归一化统计可用于核对 OpenArm v11 的 29 维观测语义；必须用回放或仿真成功率验证后才能称为 sim-to-real 可用。
- **夹爪量程/观测标定**：真实夹爪量程 0~0.042，与训练环境的开合约定可能有差异，策略可能需微调（属训练侧问题）。
- **控制器**：节点通过 ros2_control 的 Action 接口下发命令，需保证
  `*_joint_trajectory_controller` / `*_gripper_controller` 已由 GUI 或 launch 启动。
