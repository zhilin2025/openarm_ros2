# Copyright 2026 OpenArm contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Full bimanual cooperative carrying stack in Gazebo:
#   openarm_gazebo_sim (robot + workcell + grasp plugin)
#   + move_group (openarm_bimanual_moveit_config, forced to the v11 model)
#   + perception node (head-camera tabletop clustering)
#   + carry node (grasp decision, synchronized dual-arm execution).

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

THIS_PACKAGE = Path(get_package_share_directory('openarm_bimanual_coordinate'))


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_ground_truth = LaunchConfiguration('use_ground_truth')

    # --- MoveIt (v11 model to match the simulation exactly) ---------------
    moveit_config = (
        MoveItConfigsBuilder('openarm', package_name='openarm_bimanual_moveit_config')
        .robot_description(
            file_path='config/openarm_bimanual.urdf.xacro',
            mappings={'arm_type': 'v11'})
        .robot_description_semantic(file_path='config/openarm_bimanual.srdf')
        .robot_description_kinematics(
            str(THIS_PACKAGE / 'config' / 'kinematics.yaml'))
        .joint_limits(str(THIS_PACKAGE / 'config' / 'joint_limits.yaml'))
        .sensors_3d(str(THIS_PACKAGE / 'config' / 'sensors_3d.yaml'))
        .to_moveit_configs()
    )
    move_group_params = [moveit_config.to_dict(), {'use_sim_time': True}]

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_params,
    )

    # --- Perception + coordination ----------------------------------------
    perception_node = Node(
        package='openarm_bimanual_coordinate',
        executable='perception_node.py',
        name='bimanual_perception',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    carry_node = Node(
        package='openarm_bimanual_coordinate',
        executable='carry_node.py',
        name='bimanual_carry',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'use_ground_truth': use_ground_truth,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='bimanual_carry_rviz',
        output='log',
        arguments=['-d', str(THIS_PACKAGE / 'config' / 'carry.rviz')]
        if (THIS_PACKAGE / 'config' / 'carry.rviz').exists() else [],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false',
                              description='Show the Gazebo client.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz on the grasp plan.'),
        DeclareLaunchArgument('use_ground_truth', default_value='false',
                              description='Bypass perception with gazebo poses.'),
        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH',
            value=[
                FindPackageShare('openarm_bimanual_coordinate'),
                '/../lib:',
                EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value=''),
            ],
        ),
        # Simulation owns /controller_manager; move_group attaches to it.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory('openarm_gazebo_sim'))
                    / 'launch' / 'sim.launch.py')
            ),
            launch_arguments={'gui': gui, 'camera_view': 'false'}.items(),
        ),
        move_group_node,
        perception_node,
        # Give the controllers a moment to come up before the demo starts.
        TimerAction(period=10.0, actions=[rviz_node]),
        TimerAction(period=14.0, actions=[carry_node]),
    ])
