# OpenArm v11 Gazebo simulation

This package provides an offline Gazebo Classic 11 workcell for the bimanual
OpenArm v11. It reuses `openarm_description`, adds a simulated Intel RealSense
D435 RGB-D camera, and exposes the same ROS 2 controller actions used by the
hardware bringup and `openarm_rl_deploy`.

At description-generation time, the package adds minimal inertias to the four
massless v11 hand mounting links. This simulation-only repair keeps all gripper
joints during Gazebo's URDF-to-SDF conversion without changing the source
`openarm_description` model.

The default world contains a fixed work table and five dynamic objects: a mug,
water bottle, apple, cereal box, and food can. The water bottle is placed near
the table edge in the right arm's tested position-IK workspace; the other
objects are scene context and are not guaranteed to be reachable.

## Build and start

```bash
cd /home/zl/openarm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select openarm_gazebo_sim openarm
source install/setup.bash
ros2 launch openarm_gazebo_sim sim.launch.py
```

Useful launch options:

```bash
# Headless validation or training
ros2 launch openarm_gazebo_sim sim.launch.py gui:=false camera_view:=false

# Open RViz with RGB image and depth point cloud
ros2 launch openarm_gazebo_sim sim.launch.py rviz:=true
```

Do not start `openarm_bringup` at the same time. The Gazebo model owns
`/controller_manager` and supplies simulated ros2_control hardware.

## Interfaces

| Interface | Type |
| --- | --- |
| `/left_joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` |
| `/right_joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` |
| `/left_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` |
| `/right_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` |
| `/joint_states` | `sensor_msgs/msg/JointState` |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` |

## Control checks

Move the right arm:

```bash
ros2 action send_goal \
  /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [openarm_right_joint1, openarm_right_joint2, openarm_right_joint3, openarm_right_joint4, openarm_right_joint5, openarm_right_joint6, openarm_right_joint7], points: [{positions: [0.20, -0.35, 0.0, 0.80, -0.20, 0.10, 0.0], time_from_start: {sec: 3}}]}}'
```

Move the left arm:

```bash
ros2 action send_goal \
  /left_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [openarm_left_joint1, openarm_left_joint2, openarm_left_joint3, openarm_left_joint4, openarm_left_joint5, openarm_left_joint6, openarm_left_joint7], points: [{positions: [0.20, 0.35, 0.0, 0.80, -0.20, 0.10, 0.0], time_from_start: {sec: 3}}]}}'
```

Open a gripper:

```bash
ros2 action send_goal /right_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  '{command: {position: 0.035, max_effort: 7.0}}'
```

Confirm that the camera is producing data:

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_rect_raw
ros2 topic hz /camera/depth/color/points
```

The ROS camera and image_transport topics can appear after RViz, rqt image view,
or another image subscriber connects, and may be delayed while Gazebo loads the
robot. For vision integration, use `/camera/color/image_raw`,
`/camera/depth/image_rect_raw`, and `/camera/color/camera_info`. Topics below
`/camera/camera/.../compressed`, `compressedDepth`, or `theora` are derived
image_transport streams rather than the detector's primary inputs.
