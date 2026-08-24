# ROS2 Launch 文件
# 用法:
#   ros2 launch openarmx_deploy deploy.launch.py arm_side:=right checkpoint:=<path>/jgzh_sim2real.pt

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # -- 手臂选择 --
    arm_side_arg = DeclareLaunchArgument(
        "arm_side", default_value="right",
        choices=["left", "right"],
        description="Which arm to control"
    )

    # -- 模型路径 (相对路径按包 share 目录解析) --
    checkpoint_arg = DeclareLaunchArgument(
        "checkpoint",
        default_value="models/jgzh_sim2real.pt",
        description="TorchScript model path (.pt)"
    )

    # -- 物体位姿话题 --
    object_pose_topic_arg = DeclareLaunchArgument(
        "object_pose_topic",
        default_value="/yolo_detection/object_poses",
        description="Object position topic (PoseArray)"
    )
    base_frame_arg = DeclareLaunchArgument(
        "base_frame", default_value="openarm_body_link0",
        description="Frame used by the arm FK/IK"
    )
    object_timeout_arg = DeclareLaunchArgument(
        "object_timeout", default_value="1.0",
        description="Reject detections older than this many seconds"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use the Gazebo clock"
    )

    # -- 控制参数 --
    control_rate_arg = DeclareLaunchArgument(
        "control_rate", default_value="30.0",
        description="Policy loop frequency (Hz)"
    )
    max_joint_velocity_arg = DeclareLaunchArgument(
        "max_joint_velocity", default_value="0.50",
        description="Peak joint speed limit for RL trajectory segments (rad/s)"
    )
    min_arm_goal_duration_arg = DeclareLaunchArgument(
        "min_arm_goal_duration", default_value="0.25",
        description="Minimum duration of an RL arm trajectory segment (s)"
    )
    gripper_goal_epsilon_arg = DeclareLaunchArgument(
        "gripper_goal_epsilon", default_value="0.0005",
        description="Minimum gripper target change required to send a new goal (m)"
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
    approach_height_arg = DeclareLaunchArgument("approach_height", default_value="0.15")
    grasp_height_offset_arg = DeclareLaunchArgument("grasp_height_offset", default_value="0.008")
    lift_height_arg = DeclareLaunchArgument("lift_height", default_value="0.12")
    grasp_yaw_arg = DeclareLaunchArgument("grasp_yaw", default_value="0.0")
    table_z_arg = DeclareLaunchArgument(
        "table_z", default_value="0.80",
        description="Table plane height in arm base frame (for above-table clamping)"
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

    # -- 节点 --
    deploy_node = Node(
        package="openarmx_deploy",
        executable="deploy_node",
        name="openarmx_deploy",
        output="screen",
        parameters=[{
            "arm_side": LaunchConfiguration("arm_side"),
            "checkpoint": LaunchConfiguration("checkpoint"),
            "object_pose_topic": LaunchConfiguration("object_pose_topic"),
            "base_frame": LaunchConfiguration("base_frame"),
            "object_timeout": LaunchConfiguration("object_timeout"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "control_rate": LaunchConfiguration("control_rate"),
            "max_joint_velocity": LaunchConfiguration("max_joint_velocity"),
            "min_arm_goal_duration": LaunchConfiguration("min_arm_goal_duration"),
            "gripper_goal_epsilon": LaunchConfiguration("gripper_goal_epsilon"),
            "max_steps": LaunchConfiguration("max_steps"),
            "action_pos_scale": LaunchConfiguration("action_pos_scale"),
            "action_gripper_scale": LaunchConfiguration("action_gripper_scale"),
            "approach_height": LaunchConfiguration("approach_height"),
            "grasp_height_offset": LaunchConfiguration("grasp_height_offset"),
            "lift_height": LaunchConfiguration("lift_height"),
            "grasp_yaw": LaunchConfiguration("grasp_yaw"),
            "table_z": LaunchConfiguration("table_z"),
            "home_pose": LaunchConfiguration("home_pose"),
            "home_tolerance": LaunchConfiguration("home_tolerance"),
            "home_timeout": LaunchConfiguration("home_timeout"),
        }],
    )

    return LaunchDescription([
        arm_side_arg,
        checkpoint_arg,
        object_pose_topic_arg,
        base_frame_arg,
        object_timeout_arg,
        use_sim_time_arg,
        control_rate_arg,
        max_joint_velocity_arg,
        min_arm_goal_duration_arg,
        gripper_goal_epsilon_arg,
        max_steps_arg,
        action_pos_scale_arg,
        action_gripper_scale_arg,
        approach_height_arg,
        grasp_height_offset_arg,
        lift_height_arg,
        grasp_yaw_arg,
        table_z_arg,
        home_pose_arg,
        home_tolerance_arg,
        home_timeout_arg,
        deploy_node,
    ])
