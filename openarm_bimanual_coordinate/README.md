# OpenArm 双臂协同搬运（Gazebo）

在 Gazebo Classic 11 中实现 OpenArm v11 双臂协同搬运：头部深度相机识别桌面
物体 → 决定左右臂各自的抓取方式 → 双臂同步运动 → 焊接吸附 → 搬运 → 放置。

## 组成

| 部分 | 说明 |
| --- | --- |
| `src/bimanual_grasp_plugin.cpp` | Gazebo 模型插件。订阅 `/bimanual_grasp/grasp_cmd`（JSON），在指定手指链接与物体间创建/删除 fixed joint（焊接吸附），回执发布到 `/bimanual_grasp/grasp_status` |
| `scripts/perception_node.py` | 深度点云感知：TF 变换 → 裁剪 → 体素降采样 → 桌面剔除 → 聚类，发布 `vision_msgs/Detection3DArray` 到 `/bimanual_perception/objects`，RViz 标记到 `/bimanual_perception/markers` |
| `scripts/carry_node.py` | 协调器：物体模板 → 抓取决策（**双臂侧向斜抓**：左臂压物体 +X 面上部、右臂压 −X 面上部，45° 斜向接近，双臂分居物体两侧）→ **自带 scipy 有界 IK**（绕开 move_group 内 KDL 对 7 自由度冗余臂不可靠的问题）→ OMPL 关节空间规划（远距）+ 关节插值（短程）→ 双臂同步执行 → 焊接/释放 → 校验 |
| `launch/bimanual_carry.launch.py` | 一键启动：仿真（含焊接插件）+ move_group（v11 模型/运动学/关节限位覆盖）+ 感知 + 协调器 |

## 依赖的修复（已合入工作区）

- `openarm_gazebo_sim/urdf/openarm_v11_gazebo.urdf.xacro`：相机 parent 修正为
  `openarm_camera_link0`；挂载焊接插件
- `openarm_gazebo_sim/launch/sim.launch.py`：恢复 spawn `-z 0.38`
- `openarm_gazebo_sim/worlds/openarm_workcell.world`：恢复桌子、物体摆入双臂
  工作区、新增 tissue_box / storage_box 大物体、`gazebo_ros_state` 服务、
  软接触参数（防物理爆炸）、ODE iters
- `openarm_gazebo_sim/config/controllers.yaml`：joint_state_broadcaster 显式
  关节列表（排除 mimic 重复项，消除 MoveIt 刷屏报错）
- `openarm_gazebo_sim/scripts/generate_robot_description.py`：手部链接惯性
  1g → 50g（焊接质量比过大导致 ODE 失稳）
- `openarm_bimanual_moveit_config/config/openarm_bimanual.srdf`：机器人名
  `openarm_v11` → `openarm_bimanual`（名字不匹配导致 SRDF 被整体拒绝）

## 启动

```bash
# 每个新终端都需要
source /opt/ros/humble/setup.bash
source ~/openarm_ws/install/setup.bash

# 若改过代码
cd ~/openarm_ws && colcon build --symlink-install --packages-select openarm_bimanual_coordinate

# 完整启动（无头 + RViz；演示在 move_group 就绪约 15 秒后自动开始）
ros2 launch openarm_bimanual_coordinate bimanual_carry.launch.py

# 常用参数
ros2 launch openarm_bimanual_coordinate bimanual_carry.launch.py \
    gui:=true rviz:=true \        # 打开 Gazebo 客户端与 RViz
    use_ground_truth:=true        # 跳过视觉、直接用 gazebo 位姿（调试用）
```

流程：每个物体先被传送到起始位 → 感知 → 决定抓取（双臂指尖竖直下压物体
顶部两端）→ 左臂 OMPL 规划接近、双臂同步抓取 → 左手焊接吸附 → 抬升 →
分段搬运到目标位 → 释放（自由下落约 4 cm）→ 撤臂归位 → 校验 → 物体传送
到停放区，下一个物体开始。全部完成后打印 PASS/FAIL 汇总。

## 测试结果（7/7 全部通过）

| 物体 | 搬运 | 放置误差 |
| --- | --- | --- |
| water_bottle | OK | 0.2 cm |
| food_can | OK | 0.2 cm |
| cereal_box | OK | 0.2 cm |
| coffee_mug | OK | 1.3 cm |
| apple | OK | 6.1 cm |
| tissue_box | OK | 2.4 cm |
| storage_box（大） | OK | 1.6 cm |

单物体周期约 29 秒（感知/接近/抓取/焊接/抬升/搬运/释放/撤臂/归位）。
早期版本采用"顶部双按压"时圆形物体有 ODE 物理爆炸、双手在物体上方重叠；
改为**双侧斜向抓取**后爆炸消失、精度提升到厘米级。

## 已知问题与说明

1. 终端若持续刷 `Joint 'openarm_*_finger_joint2_mimic' not found`：是仿真端
   把 mimic 关节当独立关节发布、MoveIt 模型中无此名导致的警告（无害）。
   joint_state_broadcaster 已配置过滤；另外注意不要遗留多余的
   /joint_states 发布器（会重新引入刷屏）。
2. OMPL 对个别目标偶发拒绝（-2/99999），协调器已自动换 IK 种子重试。
3. 视觉感知模式（`use_ground_truth:=false`）的完整验收未完成；感知节点
   本身已就绪并发布 `/bimanual_perception/objects`。
