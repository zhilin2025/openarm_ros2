#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace openarmx_deploy {

/// 从 ROS 话题构造与训练环境一致的 29 维观测向量。
///
/// 观测组成: arm(7) + gripper(2) + obj_pos(3) + pad_center(3) + obj2pad(3)
///           + target_pos(3) + gripper_target(1) + step_frac(1) + state_oh(6) = 29
class ObservationBuilder {
 public:
  static constexpr int kStateNames = 6;       // center_xy..lift
  static constexpr int kArmJoints = 7;        // JGZH/OpenArmX/Sciurus17 均为 7-DOF
  static constexpr int kGripperJoints = 2;

  static constexpr int kObservationDim =
      kArmJoints + kGripperJoints + 3 + 3 + 3 + 3 + 1 + 1 + kStateNames;  // = 29

  ObservationBuilder();

  void updateArmQpos(const std::vector<double>& positions);
  void updateGripperQpos(const std::vector<double>& positions);
  void updateObjectPos(double x, double y, double z);
  void updateFingerpadCenter(double x, double y, double z);
  void updateTarget(const std::array<double, 3>& target_pos, double gripper_target);
  void updateState(int step_count, int max_steps,
                   const std::array<float, kStateNames>& state_one_hot);

  /// 构造完整观测向量 (29 float32)
  std::vector<float> build() const;

  /// 直接获取内部 buffer 指针（避免拷贝）
  const float* data() const { return obs_.data(); }

 private:
  std::array<float, kObservationDim> obs_{};
};

}  // namespace openarmx_deploy
