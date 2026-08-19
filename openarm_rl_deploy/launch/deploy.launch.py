# ROS2 Launch 文件
# 用法:
#   ros2 launch openarmx_deploy deploy.launch.py robot:=JGZH checkpoint:=checkpoints/jgzh_sim2real.pt

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # -- 机器人型号 --
    robot_arg = DeclareLaunchArgument(
        "robot", default_value="JGZH",
        choices=["JGZH", "OpenArmX", "Sciurus17"],
        description="Robot model"
    )

    # -- 模型路径 --
    checkpoint_arg = DeclareLaunchArgument(
        "checkpoint",
        default_value="models/jgzh_sim2real.pt",
        description="TorchScript model path (.pt)"
    )

    # -- 控制参数 --
    control_rate_arg = DeclareLaunchArgument(
        "control_rate", default_value="30",
        description="Control loop frequency (Hz)"
    )
    max_steps_arg = DeclareLaunchArgument(
        "max_steps", default_value="400",
        description="Max steps per grasp"
    )
    action_pos_scale_arg = DeclareLaunchArgument(
        "action_pos_scale", default_value="0.03",
        description="TCP position action scale (m)"
    )
    action_gripper_scale_arg = DeclareLaunchArgument(
        "action_gripper_scale", default_value="0.005",
        description="Gripper action scale"
    )

    # -- 抓取参数 --
    approach_height_arg = DeclareLaunchArgument(
        "approach_height", default_value="0.15"
    )
    grasp_height_offset_arg = DeclareLaunchArgument(
        "grasp_height_offset", default_value="0.008"
    )
    lift_height_arg = DeclareLaunchArgument(
        "lift_height", default_value="0.12"
    )
    grasp_yaw_arg = DeclareLaunchArgument(
        "grasp_yaw", default_value="0.0"
    )

    # -- 初始位姿参数 --
    home_pose_arg = DeclareLaunchArgument(
        "home_pose", default_value="",
        description="固定初始关节角 (7 个, 逗号分隔), 空=启动时记录当前位姿"
    )
    home_tolerance_arg = DeclareLaunchArgument(
        "home_tolerance", default_value="0.02",
        description="回位到位容差 (rad)"
    )
    home_timeout_arg = DeclareLaunchArgument(
        "home_timeout", default_value="30.0",
        description="回位超时 (s)"
    )

    # -- MoveIt2 参数 --
    move_group_arg = DeclareLaunchArgument(
        "move_group_name", default_value="arm",
        description="MoveIt2 规划组名 (需匹配 moveit 配置)"
    )
    use_moveit_arg = DeclareLaunchArgument(
        "use_moveit", default_value="true",
        description="回位是否使用 MoveIt2 避障规划"
    )
    moveit_velocity_scale_arg = DeclareLaunchArgument(
        "moveit_velocity_scale", default_value="0.3",
        description="回位速度缩放 (0~1)"
    )

    # -- 节点 --
    deploy_node = Node(
        package="openarmx_deploy",
        executable="deploy_node",
        name="openarmx_deploy",
        output="screen",
        parameters=[{
            "robot": LaunchConfiguration("robot"),
            "checkpoint": LaunchConfiguration("checkpoint"),
            "control_rate": LaunchConfiguration("control_rate"),
            "max_steps": LaunchConfiguration("max_steps"),
            "action_pos_scale": LaunchConfiguration("action_pos_scale"),
            "action_gripper_scale": LaunchConfiguration("action_gripper_scale"),
            "approach_height": LaunchConfiguration("approach_height"),
            "grasp_height_offset": LaunchConfiguration("grasp_height_offset"),
            "lift_height": LaunchConfiguration("lift_height"),
            "grasp_yaw": LaunchConfiguration("grasp_yaw"),
            "home_pose": LaunchConfiguration("home_pose"),
            "home_tolerance": LaunchConfiguration("home_tolerance"),
            "home_timeout": LaunchConfiguration("home_timeout"),
            "move_group_name": LaunchConfiguration("move_group_name"),
            "use_moveit": LaunchConfiguration("use_moveit"),
            "moveit_velocity_scale": LaunchConfiguration("moveit_velocity_scale"),
        }],
    )

    return LaunchDescription([
        robot_arg,
        checkpoint_arg,
        control_rate_arg,
        max_steps_arg,
        action_pos_scale_arg,
        action_gripper_scale_arg,
        approach_height_arg,
        grasp_height_offset_arg,
        lift_height_arg,
        grasp_yaw_arg,
        home_pose_arg,
        home_tolerance_arg,
        home_timeout_arg,
        move_group_arg,
        use_moveit_arg,
        moveit_velocity_scale_arg,
        deploy_node,
    ])
