场景 1：开发和调试
# 终端 1 - 启动完整系统
ros2 launch openarm_bimanual_moveit_config demo.launch.py



场景 2：只做运动规划（不启动硬件）
# 终端 1 - 启动 move_group
ros2 launch openarm_bimanual_moveit_config move_group.launch.py
# 终端 2 - 启动 RViz 可视化
ros2 launch openarm_bimanual_moveit_config moveit_rviz.launch.py
# 需要：move_group 已启动 + robot_description 已加载



场景 3：修改机器人配置
# 启动 MoveIt Setup Assistant
ros2 launch openarm_bimanual_moveit_config setup_assistant.launch.py
# 这是独立的工具，用来修改 SRDF、添加移动组等



场景 4：特殊场景需求
# 只发布虚拟关节变换
ros2 launch openarm_bimanual_moveit_config static_virtual_joint_tfs.launch.py
