# Copyright 2025 Enactic, Inc.
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
from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_robot_description(
    context: LaunchContext,
    description_package,
    description_file,
    arm_type,
    use_fake_hardware,
    right_can_interface,
    left_can_interface,
    arm_prefix,
    motor_backend,
    robstride_master_id,
    robstride_joint_ids,
    robstride_joint_types,
    robstride_gripper_id,
    robstride_gripper_type,
    auto_return_to_zero_on_activate,
    limit_margin,
    limit_stop_margin,
    limit_decel_factor,
    zero_torque_kd,
):
    """Render Xacro and return XML string."""
    description_package_str = context.perform_substitution(description_package)
    description_file_str = context.perform_substitution(description_file)
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    right_can_interface_str = context.perform_substitution(right_can_interface)
    left_can_interface_str = context.perform_substitution(left_can_interface)
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

    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "urdf",
        "robot",
        description_file_str,
    )

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "true",
            "use_fake_hardware": use_fake_hardware_str,
            "ros2_control": "true",
            "left_can_interface": left_can_interface_str,
            "right_can_interface": right_can_interface_str,
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
            # arm_prefix unused inside xacro but kept for completeness
        },
    ).toprettyxml(indent="  ")

    return robot_description


def robot_nodes_spawner(
    context: LaunchContext,
    description_package,
    description_file,
    arm_type,
    use_fake_hardware,
    controllers_file,
    right_can_interface,
    left_can_interface,
    arm_prefix,
    motor_backend,
    robstride_master_id,
    robstride_joint_ids,
    robstride_joint_types,
    robstride_gripper_id,
    robstride_gripper_type,
    auto_return_to_zero_on_activate,
    limit_margin,
    limit_stop_margin,
    limit_decel_factor,
    zero_torque_kd,
):
    robot_description = generate_robot_description(
        context,
        description_package,
        description_file,
        arm_type,
        use_fake_hardware,
        right_can_interface,
        left_can_interface,
        arm_prefix,
        motor_backend,
        robstride_master_id,
        robstride_joint_ids,
        robstride_joint_types,
        robstride_gripper_id,
        robstride_gripper_type,
        auto_return_to_zero_on_activate,
        limit_margin,
        limit_stop_margin,
        limit_decel_factor,
        zero_torque_kd,
    )

    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {"robot_description": robot_description}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub_node, control_node]


def controller_spawner(context: LaunchContext, robot_controller):
    robot_controller_str = context.perform_substitution(robot_controller)

    if robot_controller_str == "forward_position_controller":
        left = "left_forward_position_controller"
        right = "right_forward_position_controller"
    elif robot_controller_str == "joint_trajectory_controller":
        left = "left_joint_trajectory_controller"
        right = "right_joint_trajectory_controller"
    else:
        raise ValueError(f"Unknown robot_controller: {robot_controller_str}")

    return [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[left, right, "-c", "/controller_manager"],
        )
    ]


def write_zero_torque_param_file(context, gravity_scale, zero_torque_kd):
    gravity_scale_value = context.perform_substitution(gravity_scale)
    zero_torque_kd_value = context.perform_substitution(zero_torque_kd)
    contents = (
        "left_zero_torque_controller:\n"
        "  ros__parameters:\n"
        f"    gravity_scale: {gravity_scale_value}\n"
        f"    kd: {zero_torque_kd_value}\n"
        "right_zero_torque_controller:\n"
        "  ros__parameters:\n"
        f"    gravity_scale: {gravity_scale_value}\n"
        f"    kd: {zero_torque_kd_value}\n"
    )
    param_path = os.path.join(tempfile.gettempdir(), "openarm_zero_torque_params.yaml")
    with open(param_path, "w", encoding="utf-8") as handle:
        handle.write(contents)
    return param_path


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="openarm_description",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="v10.urdf.xacro",
        ),
        DeclareLaunchArgument("arm_type", default_value="v10"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["forward_position_controller",
                     "joint_trajectory_controller"],
        ),
        DeclareLaunchArgument(
            "runtime_config_package", default_value="openarm_bringup"
        ),
        DeclareLaunchArgument("arm_prefix", default_value=""),
        DeclareLaunchArgument("right_can_interface", default_value="can0"),
        DeclareLaunchArgument("left_can_interface", default_value="can1"),
        DeclareLaunchArgument(
            "motor_backend",
            default_value="robstride",
            choices=["damiao", "robstride"],
        ),
        DeclareLaunchArgument("robstride_master_id", default_value="253"),
        DeclareLaunchArgument(
            "robstride_joint_ids", default_value="1,2,3,4,5,6,7"
        ),
        DeclareLaunchArgument(
            "robstride_joint_types", default_value="3,3,6,6,0,0,0"
        ),
        DeclareLaunchArgument("robstride_gripper_id", default_value="8"),
        DeclareLaunchArgument("robstride_gripper_type", default_value="0"),
        DeclareLaunchArgument(
            "auto_return_to_zero_on_activate",
            default_value="false",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "limit_margin",
            default_value="0.1",
        ),
        DeclareLaunchArgument(
            "limit_stop_margin",
            default_value="0.02",
        ),
        DeclareLaunchArgument(
            "limit_decel_factor",
            default_value="0.2",
        ),
        DeclareLaunchArgument(
            "zero_torque_kd",
            default_value="0.3",
        ),
        DeclareLaunchArgument(
            "gravity_scale",
            default_value="1.0",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="openarm_v10_bimanual_controllers.yaml",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    arm_type = LaunchConfiguration("arm_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    right_can_interface = LaunchConfiguration("right_can_interface")
    left_can_interface = LaunchConfiguration("left_can_interface")
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

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config",
         "v10_controllers", controllers_file]
    )

    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[
            description_package,
            description_file,
            arm_type,
            use_fake_hardware,
            controllers_file,
            right_can_interface,
            left_can_interface,
            arm_prefix,
            motor_backend,
            robstride_master_id,
            robstride_joint_ids,
            robstride_joint_types,
            robstride_gripper_id,
            robstride_gripper_type,
            auto_return_to_zero_on_activate,
            limit_margin,
            limit_stop_margin,
            limit_decel_factor,
            zero_torque_kd,
        ],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    controller_spawner_func = OpaqueFunction(
        function=controller_spawner, args=[robot_controller])

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_gripper_controller",
                   "right_gripper_controller", "-c", "/controller_manager"],
    )

    zero_torque_spawner = OpaqueFunction(
        function=lambda context: [Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "left_zero_torque_controller",
                "right_zero_torque_controller",
                "-c",
                "/controller_manager",
                "--param-file",
                write_zero_torque_param_file(context, gravity_scale, zero_torque_kd),
                "--inactive",
            ],
        )]
    )

    delayed_jsb = TimerAction(period=2.0, actions=[jsb_spawner])
    delayed_arm_ctrl = TimerAction(
        period=1.0, actions=[controller_spawner_func])
    delayed_gripper = TimerAction(period=1.0, actions=[gripper_spawner])
    delayed_zero_torque = TimerAction(period=1.0, actions=[zero_torque_spawner])

    moveit_config = MoveItConfigsBuilder(
        "openarm", package_name="openarm_bimanual_moveit_config"
    ).to_moveit_configs()

    moveit_params = moveit_config.to_dict()

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
    )

    rviz_cfg = os.path.join(
        get_package_share_directory(
            "openarm_bimanual_moveit_config"), "config", "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_cfg],
        parameters=[moveit_params],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_nodes_spawner_func,
            delayed_jsb,
            delayed_arm_ctrl,
            delayed_gripper,
            delayed_zero_torque,
            run_move_group_node,
            rviz_node,
        ]
    )
