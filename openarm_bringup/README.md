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
# ros2 launch openarm_bringup openarm.bimanual.launch.py 
```
```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=false motor_backend:=robstride right_can_interface:=can0 left_can_interface:=can1 robstride_master_id:=253 robstride_joint_ids:=1,2,3,4,5,6,7 robstride_joint_types:=3,3,6,6,0,0,0 robstride_gripper_id:=8 robstride_gripper_type:=0 auto_return_to_zero_on_activate:=false gravity_scale:=0.0 zero_torque_kd:=0.5
```

控制左臂运动到指定位置
```bash
ros2 action send_goal /left_joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{trajectory: {joint_names: ["openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3", "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6", "openarm_left_joint7"], points: [{positions: [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15], time_from_start: {sec: 3, nanosec: 0}}]}}'
```
控制右臂运动到指定位置
```bash
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{trajectory: {joint_names: ["openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3", "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6", "openarm_right_joint7"], points: [{positions: [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15], time_from_start: {sec: 3, nanosec: 0}}]}}'
```
控制右臂夹爪到指定位置
```bash
ros2 action send_goal /right_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.04, max_effort: 1.0}}"
```
控制左臂夹爪到指定位置
```bash
ros2 action send_goal /left_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.04, max_effort: 1.0}}"
```

# 单臂运动控制
启动单臂运动控制脚本
```bash
# ros2 launch openarm_bringup openarm.launch.py
```
控制单臂运动到指定位置
```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{trajectory: {joint_names: ["openarm_joint1", "openarm_joint2", "openarm_joint3", "openarm_joint4", "openarm_joint5", "openarm_joint6", "openarm_joint7"], points: [{positions: [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15], time_from_start: {sec: 3, nanosec: 0}}]}}'
```
```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{trajectory: {joint_names: ["openarm_joint1", "openarm_joint2", "openarm_joint3", "openarm_joint4", "openarm_joint5", "openarm_joint6", "openarm_joint7"], points: [{positions: [0.15, 0.15, 0.15, 0.55, 0.15, 0.55, 0.15], time_from_start: {sec: 3, nanosec: 0}}]}}'
```
控制夹爪到指定位置(用的是gripper_controller控制器，不会做路径插补，直接运动到指定位置，速度较快)
```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.04, max_effort: 1.0}}"
```

多功能调试：(先不要开启自动回零，执行前需要先手动回零，然后再执行下面发布控制指令)
```bash
ros2 launch openarm_bringup openarm.launch.py use_fake_hardware:=false motor_backend:=robstride can_interface:=can0 robstride_master_id:=253 robstride_joint_ids:=1,2,3,4,5,6,7 robstride_joint_types:=3,3,6,6,0,0,0 robstride_gripper_id:=8 robstride_gripper_type:=0 auto_return_to_zero_on_activate:=false gravity_scale:=0.0 zero_torque_kd:=0.5
```
