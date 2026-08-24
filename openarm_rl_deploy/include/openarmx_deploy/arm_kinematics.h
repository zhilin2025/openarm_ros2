#pragma once

#include <array>
#include <vector>

namespace openarmx_deploy {

/// OpenArmX v11 7-DOF 机械臂正/逆运动学（进程内求解，无 Pinocchio / MoveIt 依赖）。
///
/// 关节链参数硬编码自 openarm_description/urdf/robot/v11.urdf.xacro 的
/// v11_arm_left / v11_arm_right 宏（每关节 origin xyz / rpy / axis），末端帧为
/// openarm_left_link7 / openarm_right_link7。
class ArmKinematics {
 public:
  enum class Side { kLeft, kRight };

  explicit ArmKinematics(Side side = Side::kRight);

  /// 正运动学：7 关节角 -> (TCP 位置, 3x3 旋转矩阵 row-major)
  void forward(const std::vector<double>& q,
               std::array<double, 3>& pos,
               std::array<double, 9>& R) const;

  /// 只返回 TCP 位置（便捷方法）
  std::array<double, 3> forwardPos(const std::vector<double>& q) const;

  /// 位置 DLS 数值 IK：求 q 使 TCP 到达 target_pos（7-DOF 冗余，只约束位置）。
  /// @return 是否收敛到 eps；即使未收敛 out_q 仍为裁剪限位后的可行近似解。
  bool inversePosition(const std::array<double, 3>& target_pos,
                       const std::vector<double>& q_init,
                       std::vector<double>& out_q) const;

  const std::array<double, 7>& lower() const { return lower_; }
  const std::array<double, 7>& upper() const { return upper_; }

 private:
  struct JointParam {
    std::array<double, 3> xyz;   // origin 平移
    std::array<double, 3> rpy;   // origin 旋转 (rad, 固定轴 XYZ)
    std::array<double, 3> axis;  // 关节轴 (单位向量)
  };

  std::array<JointParam, 7> joints_;
  std::array<double, 7> lower_;
  std::array<double, 7> upper_;
};

}  // namespace openarmx_deploy
