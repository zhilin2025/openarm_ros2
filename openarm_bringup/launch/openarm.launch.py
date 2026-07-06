# Copyright 2025 Enactic, Inc.
# Copyright 2024 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def write_zero_torque_param_file(context, gravity_scale, zero_torque_kd):
    gravity_scale_value = context.perform_substitution(gravity_scale)
    zero_torque_kd_value = context.perform_substitution(zero_torque_kd)
    contents = (
        "zero_torque_controller:\n"
        "  ros__parameters:\n"
        f"    gravity_scale: {gravity_scale_value}\n"
        f"    kd: {zero_torque_kd_value}\n"
    )
    param_path = os.path.join(tempfile.gettempdir(), "openarm_zero_torque_params.yaml")
    with open(param_path, "w", encoding="utf-8") as handle:
        handle.write(contents)
    return param_path


## 将 Xacro 文件转换为 URDF 字符串，是连接 “Xacro 配置” 和 “ROS 2 节点” 的桥梁
def generate_robot_description(context: LaunchContext, description_package, description_file,
                               arm_type, use_fake_hardware, can_interface, arm_prefix,
                               motor_backend, robstride_master_id, robstride_joint_ids,
                               robstride_joint_types, robstride_gripper_id,
                               robstride_gripper_type,
                               auto_return_to_zero_on_activate, limit_margin,
                               limit_stop_margin, limit_decel_factor,
                               zero_torque_kd):
    """Generate robot description using xacro processing."""
    """返回值是一个包含 robot_description（URDF/XML）的字符串，用于作为参数传给 robot_state_publisher 和 ros2_control_node，供后续启动节点读取机器人模型与 ros2_control 配置。"""

    # Substitute launch configuration values
    description_package_str = context.perform_substitution(description_package)
    description_file_str = context.perform_substitution(description_file)
    arm_type_str = context.perform_substitution(arm_type)

    if arm_type_str == "v11" and description_file_str == "v10.urdf.xacro":
        description_file_str = "v11.urdf.xacro"

    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    can_interface_str = context.perform_substitution(can_interface)
    arm_prefix_str = context.perform_substitution(arm_prefix)
    motor_backend_str = context.perform_substitution(motor_backend)
    robstride_master_id_str = context.perform_substitution(robstride_master_id)
    robstride_joint_ids_str = context.perform_substitution(robstride_joint_ids)
    robstride_joint_types_str = context.perform_substitution(robstride_joint_types)
    robstride_gripper_id_str = context.perform_substitution(robstride_gripper_id)
    robstride_gripper_type_str = context.perform_substitution(robstride_gripper_type)
    auto_return_to_zero_on_activate_str = context.perform_substitution(
        auto_return_to_zero_on_activate)
    limit_margin_str = context.perform_substitution(limit_margin)
    limit_stop_margin_str = context.perform_substitution(limit_stop_margin)
    limit_decel_factor_str = context.perform_substitution(limit_decel_factor)
    zero_torque_kd_str = context.perform_substitution(zero_torque_kd)

    # Build xacro file path
    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "urdf", "robot", description_file_str
    )

    # Process xacro with required arguments
    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "false",
            "use_fake_hardware": use_fake_hardware_str,
            "ros2_control": "true",
            "can_interface": can_interface_str,
            "arm_prefix": arm_prefix_str,
            "motor_backend": motor_backend_str,
            "robstride_master_id": robstride_master_id_str,
            "robstride_joint_ids": robstride_joint_ids_str,
            "robstride_joint_types": robstride_joint_types_str,
            "robstride_gripper_id": robstride_gripper_id_str,
            "robstride_gripper_type": robstride_gripper_type_str,
            "auto_return_to_zero_on_activate": auto_return_to_zero_on_activate_str,
            "limit_margin": limit_margin_str,
            "limit_stop_margin": limit_stop_margin_str,
            "limit_decel_factor": limit_decel_factor_str,
            "zero_torque_kd": zero_torque_kd_str,
        }
    ).toprettyxml(indent="  ")

    return robot_description


def robot_nodes_spawner(context: LaunchContext, description_package, description_file,
                        arm_type, use_fake_hardware, controllers_file, can_interface,
                        arm_prefix, motor_backend, robstride_master_id,
                        robstride_joint_ids, robstride_joint_types,
                        robstride_gripper_id, robstride_gripper_type,
                        auto_return_to_zero_on_activate, limit_margin,
                        limit_stop_margin, limit_decel_factor,
                        zero_torque_kd):
    """Spawn both robot state publisher and control nodes with shared robot description."""

    # Generate robot description once
    robot_description = generate_robot_description(
        context, description_package, description_file, arm_type,
        use_fake_hardware, can_interface, arm_prefix,
        motor_backend, robstride_master_id, robstride_joint_ids,
        robstride_joint_types, robstride_gripper_id, robstride_gripper_type,
        auto_return_to_zero_on_activate, limit_margin, limit_stop_margin,
        limit_decel_factor, zero_torque_kd
    )

    # Get controllers file path
    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {"robot_description": robot_description}

    # Robot state publisher node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    # Control node， 提供硬件接口和 controller_manager 服务
    # 启动ros2_control，包含 controller_manager、硬件接口（hardware_interface）插件、以及根据 robot_description / controllers 配置初始化的底层驱动逻辑，“承载”所有控制器插件和与硬件交互的进程
    # 下面的三个spawner 节点调用 controller_manager 的服务去 load/start 指定控制器，controller_manager 在自己的进程内实例化并运行这些控制器插件。
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub_node, control_node]


def generate_launch_description():
    """Generate launch description for OpenArm unimanual configuration."""

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="openarm_description",
            description="Description package with robot URDF/xacro files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="v10.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "arm_type",
            default_value="v10",
            description="Type of arm (e.g., v10).",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            # default_value="false",
            default_value="true",
            description="Use fake hardware instead of real hardware.",
        ), 
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["forward_position_controller",
                     "joint_trajectory_controller"],
            description="Robot controller to start.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="openarm_bringup",
            description="Package with the controller's configuration in config folder.",
        ),
        DeclareLaunchArgument(
            "arm_prefix",
            default_value="",
            description="Prefix for the arm.",
        ),
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="CAN interface to use.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="openarm_v10_controllers.yaml",
            description="Controllers file(s) to use. Can be a single file or comma-separated list of files.",
        ),
        DeclareLaunchArgument(
            "motor_backend",
            default_value="damiao",
            choices=["damiao", "robstride"],
            description="Motor backend type used by openarm_hardware.",
        ),
        DeclareLaunchArgument(
            "robstride_master_id",
            default_value="253",
            description="RobStride master CAN id in decimal.",
        ),
        DeclareLaunchArgument(
            "robstride_joint_ids",
            default_value="1,2,3,4,5,6,7",
            description="Comma-separated RobStride joint motor ids.",
        ),
        DeclareLaunchArgument(
            "robstride_joint_types",
            default_value="3,3,6,6,0,0,0",
            description="Comma-separated RobStride actuator types for 7 joints.",
        ),
        DeclareLaunchArgument(
            "robstride_gripper_id",
            default_value="8",
            description="RobStride gripper motor id.",
        ),
        DeclareLaunchArgument(
            "robstride_gripper_type",
            default_value="0",
            description="RobStride gripper actuator type.",
        ),
        DeclareLaunchArgument(
            "auto_return_to_zero_on_activate",
            default_value="false",
            choices=["true", "false"],
            description="Whether to auto-command return-to-zero during hardware activation.",
        ),
        DeclareLaunchArgument(
            "limit_margin",
            default_value="0.1",
            description="Soft limit margin for effort-mode protection (radians).",
        ),
        DeclareLaunchArgument(
            "limit_stop_margin",
            default_value="0.02",
            description="Hard stop margin for effort-mode protection (radians).",
        ),
        DeclareLaunchArgument(
            "limit_decel_factor",
            default_value="0.2",
            description="Torque scale factor inside the limit margin (0-1).",
        ),
        DeclareLaunchArgument(
            "zero_torque_kd",
            default_value="0.3",
            description="Damping gain used by the hardware in effort mode.",
        ),
        DeclareLaunchArgument(
            "gravity_scale",
            default_value="1.0",
            description="Gravity compensation scale for zero torque controller (0-1).",
        ),
    ]

    # Initialize launch configurations
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    arm_type = LaunchConfiguration("arm_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    can_interface = LaunchConfiguration("can_interface")
    arm_prefix = LaunchConfiguration("arm_prefix")
    motor_backend = LaunchConfiguration("motor_backend")
    robstride_master_id = LaunchConfiguration("robstride_master_id")
    robstride_joint_ids = LaunchConfiguration("robstride_joint_ids")
    robstride_joint_types = LaunchConfiguration("robstride_joint_types")
    robstride_gripper_id = LaunchConfiguration("robstride_gripper_id")
    robstride_gripper_type = LaunchConfiguration("robstride_gripper_type")
    auto_return_to_zero_on_activate = LaunchConfiguration(
        "auto_return_to_zero_on_activate")
    limit_margin = LaunchConfiguration("limit_margin")
    limit_stop_margin = LaunchConfiguration("limit_stop_margin")
    limit_decel_factor = LaunchConfiguration("limit_decel_factor")
    zero_torque_kd = LaunchConfiguration("zero_torque_kd")
    gravity_scale = LaunchConfiguration("gravity_scale")
    # Configuration file paths
    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config",
         "v10_controllers", controllers_file]
    )

    # Robot nodes spawner (both state publisher and control)
    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[description_package, description_file, arm_type,
              use_fake_hardware, controllers_file, can_interface, arm_prefix,
              motor_backend, robstride_master_id, robstride_joint_ids,
              robstride_joint_types, robstride_gripper_id,
              robstride_gripper_type, auto_return_to_zero_on_activate,
              limit_margin, limit_stop_margin, limit_decel_factor,
              zero_torque_kd]
    )
    # RViz configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz",
         "arm_only.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Joint state broadcaster spawner
    # 加载并启动名为 joint_state_broadcaster 的控制器。该控制器发布 /joint_states（机器人各关节的状态），供 tf（robot_state_publisher）和上层控制器/监视组件使用
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    # Controller spawners
    # 使用 launch 参数 robot_controller（默认 joint_trajectory_controller）去加载并启动移动臂的主控制器（通常是 joint_trajectory_controller）。该控制器负责接收轨迹命令（通常暴露 /follow_joint_trajectory 动作服务器或相应话题/接口），因此你可以用 ros2 action send_goal 给机械臂发送轨迹目标
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "-c", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )

    zero_torque_controller_spawner = OpaqueFunction(
        function=lambda context: [Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "zero_torque_controller",
                "-c",
                "/controller_manager",
                "--param-file",
                write_zero_torque_param_file(context, gravity_scale, zero_torque_kd),
                "--inactive",
            ],
        )]
    )

    # Timing and sequencing
    # 确保 ros2_control_node / controller_manager 已就绪后再加载控制器，避免启动顺序问题（整个控制节点启动之后再加载各个控制器）
    delayed_joint_state_broadcaster = TimerAction(
        period=1.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_robot_controller = TimerAction(
        period=1.0,
        actions=[robot_controller_spawner],
    )
    delayed_gripper_controller = TimerAction(
        period=1.0,
        actions=[gripper_controller_spawner],
    )

    delayed_zero_torque_controller = TimerAction(
        period=1.0,
        actions=[zero_torque_controller_spawner],
    )

    return LaunchDescription(
        declared_arguments + [
            robot_nodes_spawner_func,
            rviz_node,
        ] +
        [
            delayed_joint_state_broadcaster,
            delayed_robot_controller,
            delayed_gripper_controller,
            delayed_zero_torque_controller,
        ]
    )
