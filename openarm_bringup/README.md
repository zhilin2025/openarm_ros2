# OpenArm Bringup

This package provides launch files to bring up the OpenArm robot system.

## Quick Start

Launch the OpenArm with v1.0 configuration and fake hardware:

```bash
ros2 launch openarm_bringup openarm.launch.py arm_type:=v10 hardware_type:=real
```

## Launch Files

- `openarm.launch.py` - Single arm configuration
- `openarm.bimanual.launch.py` - Dual arm configuration

## Key Parameters

- `arm_type` - Arm type (default: v10)
- `hardware_type` - Use real/mock/mujoco hardware (default: real)
- `can_interface` - CAN interface to use (default: can0)
- `robot_controller` - Controller type: `joint_trajectory_controller` or `forward_position_controller`

## What Gets Launched

- Robot state publisher
- Controller manager with ros2_control
- Joint state broadcaster
- Robot controller (joint trajectory or forward position)
- Gripper controller
- RViz2 visualization

## 使用例子
# 双臂运动控制
启动双臂运动控制脚本
```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py 
```
控制左臂运动到指定位置
```bash
ros2 action send_goal /left_joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{trajectory: {joint_names: ["openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3", "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6", "openarm_left_joint7"], points: [{positions: [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15], time_from_start: {sec: 3, nanosec: 0}}]}}'
```

# 单臂运动控制
启动单臂运动控制脚本
```bash
ros2 launch openarm_bringup openarm.launch.py
```
控制单臂运动到指定位置
```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{trajectory: {joint_names: ["openarm_joint1", "openarm_joint2", "openarm_joint3", "openarm_joint4", "openarm_joint5", "openarm_joint6", "openarm_joint7"], points: [{positions: [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15], time_from_start: {sec: 3, nanosec: 0}}]}}'
```
