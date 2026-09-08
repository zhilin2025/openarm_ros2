#pragma once

#include <array>
#include <string>
#include <vector>

namespace openarmx_deploy {

/// 六阶段抓取状态机，与训练环境 OpenArmXVisualGraspEnv 完全一致
/// center_xy → approach → descend → close → settle → lift
class GraspStateMachine {
 public:
  /// @param object_pos 物体初始 3D 位置 [x, y, z]
  GraspStateMachine(const std::array<double, 3>& object_pos,
                    double approach_height = 0.15,
                    double grasp_height_offset = 0.008,
                    double lift_height = 0.12,
                    double table_clearance = 0.02,
                    double table_z = 0.80,
                    double grasp_yaw = 0.0,
                    int close_steps = 30,
                    int settle_steps = 20,
                    int lift_steps = 40,
                    double gripper_open = -0.042,
                    double gripper_close = 0.0);

  /// 根据位姿误差推进状态机
  void update(double position_error, double xy_error, double z_error);

  /// 获取当前阶段的目标 TCP 位置、旋转矩阵及夹爪目标
  void getTarget(const std::array<double, 3>& current_object_pos,
                 std::array<double, 3>& target_pos,
                 std::array<double, 9>& target_xmat,
                 double& gripper_qpos) const;

  /// 返回当前状态的 one-hot 编码 (6 维)
  std::array<float, 6> oneHot() const;

  /// 当前状态名称
  const std::string& state() const { return state_; }

  /// 当前阶段步数
  int stateSteps() const { return state_steps_; }

  /// 是否已完成（lift 阶段结束后）
  bool isDone() const;

 private:
  void clampAboveTable(std::array<double, 3>& target) const;

  std::string state_ = "center_xy";
  int state_steps_ = 0;

  std::array<double, 3> object_start_pos_;
  double approach_height_;
  double grasp_height_offset_;
  double lift_height_;
  double table_clearance_;
  double table_z_;
  int close_steps_;
  int settle_steps_;
  int lift_steps_;
  double gripper_open_;
  double gripper_close_;

  // 目标姿态: top-down 抓取, 绕 Z 轴旋转 grasp_yaw
  std::array<double, 9> target_xmat_;  // 3x3 row-major
  std::array<double, 3> xy_target_;
};

}  // namespace openarmx_deploy
