# Copyright 2026 OpenArm contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("openarm_gazebo_sim")
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    paused = LaunchConfiguration("paused")
    verbose = LaunchConfiguration("verbose")
    camera_view = LaunchConfiguration("camera_view")
    rviz = LaunchConfiguration("rviz")

    xacro_file = PathJoinSubstitution(
        [package_share, "urdf", "openarm_v11_gazebo.urdf.xacro"]
    )
    robot_description = ParameterValue(
        Command(
            [
                "python3 ",
                PathJoinSubstitution(
                    [package_share, "scripts", "generate_robot_description.py"]
                ),
                " ",
                xacro_file,
            ]
        ),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "world": world,
            "gui": gui,
            "pause": paused,
            "verbose": verbose,
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_openarm_v11",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "openarm_v11",
            "-package_to_model",
            # v11 body mesh extends ~0.379 m below its root frame; previously
            # provided by the world_to_openarm joint, now handled at spawn.
            # "-z", "0.38",
        ],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    motion_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_joint_trajectory_controller",
            "right_joint_trajectory_controller",
            "left_gripper_controller",
            "right_gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--activate-as-group",
        ],
        output="screen",
    )

    camera_viewer = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="openarm_camera_view",
                arguments=["/camera/color/image_raw"],
                condition=IfCondition(camera_view),
                output="screen",
            )
        ],
    )

    rviz_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="openarm_sim_rviz",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [package_share, "rviz", "openarm_camera.rviz"]
                    ),
                ],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(rviz),
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [package_share, "worlds", "openarm_workcell.world"]
                ),
                description="Gazebo world file.",
            ),
            DeclareLaunchArgument(
                "gui", default_value="true", description="Start the Gazebo client."
            ),
            DeclareLaunchArgument(
                "paused",
                default_value="false",
                description="Start Gazebo with physics paused.",
            ),
            DeclareLaunchArgument(
                "verbose",
                default_value="false",
                description="Enable verbose Gazebo server logging.",
            ),
            DeclareLaunchArgument(
                "camera_view",
                default_value="true",
                description="Open rqt_image_view for the simulated D435 RGB stream.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Open RViz with robot, RGB image, and depth point cloud displays.",
            ),
            SetEnvironmentVariable(
                name="GAZEBO_MODEL_PATH",
                value=[
                    FindPackageShare("openarm_description"),
                    "/..:",
                    FindPackageShare("realsense2_description"),
                    "/..:",
                    EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
                ],
            ),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[joint_state_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_spawner,
                    on_exit=[motion_controller_spawner],
                )
            ),
            camera_viewer,
            rviz_node,
        ]
    )
