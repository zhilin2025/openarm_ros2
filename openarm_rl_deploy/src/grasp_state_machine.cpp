#include "openarmx_deploy/grasp_state_machine.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace openarmx_deploy {

namespace {
constexpr double kXYTol = 0.015;
constexpr double kZTol = 0.02;
constexpr double kPosTol = 0.03;
constexpr int kFallback = 200;
}  // namespace

GraspStateMachine::GraspStateMachine(const std::array<double, 3>& object_pos,
                                     double approach_height,
                                     double grasp_height_offset,
                                     double lift_height,
                                     double table_clearance,
                                     double table_z,
                                     double grasp_yaw,
                                     int close_steps,
                                     int settle_steps,
                                     int lift_steps,
                                     double gripper_open,
                                     double gripper_close)
    : object_start_pos_(object_pos),
      approach_height_(approach_height),
      grasp_height_offset_(grasp_height_offset),
      lift_height_(lift_height),
      table_clearance_(table_clearance),
      table_z_(table_z),
      close_steps_(close_steps),
      settle_steps_(settle_steps),
      lift_steps_(lift_steps),
      gripper_open_(gripper_open),
      gripper_close_(gripper_close) {
  // 目标姿态: top-down 抓取, 绕 Z 轴旋转 grasp_yaw
  double cy = std::cos(grasp_yaw);
  double sy = std::sin(grasp_yaw);
  target_xmat_ = {
      cy, -sy, 0.0,
      sy,  cy, 0.0,
      0.0, 0.0, 1.0
  };

  xy_target_ = object_start_pos_;
  xy_target_[2] = object_start_pos_[2] + approach_height_;
}

void GraspStateMachine::update(double position_error, double xy_error, double z_error) {
  state_steps_++;

  if (state_ == "center_xy" && (xy_error < kXYTol || state_steps_ > kFallback)) {
    state_ = "approach";
    state_steps_ = 0;
  } else if (state_ == "approach" &&
             (position_error < kPosTol || state_steps_ > kFallback)) {
    state_ = "descend";
    state_steps_ = 0;
  } else if (state_ == "descend" &&
             ((xy_error < kXYTol && z_error < kZTol) || state_steps_ > kFallback)) {
    state_ = "close";
    state_steps_ = 0;
  } else if (state_ == "close" && state_steps_ > close_steps_) {
    state_ = "settle";
    state_steps_ = 0;
  } else if (state_ == "settle" && state_steps_ > settle_steps_) {
    state_ = "lift";
    state_steps_ = 0;
  }
}

void GraspStateMachine::getTarget(const std::array<double, 3>& current_object_pos,
                                  std::array<double, 3>& target_pos,
                                  std::array<double, 9>& target_xmat,
                                  double& gripper_qpos) const {
  const auto& obj = current_object_pos;
  target_xmat = target_xmat_;

  auto setTarget = [&](double dx, double dy, double dz, double grip) {
    target_pos = {obj[0] + dx, obj[1] + dy, obj[2] + dz};
    clampAboveTable(target_pos);
    gripper_qpos = grip;
  };

  if (state_ == "center_xy") {
    target_pos = xy_target_;
    gripper_qpos = gripper_open_;
    return;
  }

  if (state_ == "approach") {
    setTarget(0.0, 0.0, approach_height_, gripper_open_);
    return;
  }

  if (state_ == "descend") {
    setTarget(0.0, 0.0, grasp_height_offset_, gripper_open_);
    return;
  }

  if (state_ == "close") {
    double fraction = std::min(1.0, static_cast<double>(state_steps_) /
                                        std::max(1, close_steps_));
    double grip = gripper_open_ + fraction * (gripper_close_ - gripper_open_);
    setTarget(0.0, 0.0, grasp_height_offset_, grip);
    return;
  }

  if (state_ == "settle") {
    setTarget(0.0, 0.0, grasp_height_offset_, gripper_close_);
    return;
  }

  // lift
  double fraction = std::min(1.0, static_cast<double>(state_steps_) /
                                      std::max(1, lift_steps_));
  double z = grasp_height_offset_ + fraction * (lift_height_ - grasp_height_offset_);
  setTarget(0.0, 0.0, z, gripper_close_);
}

void GraspStateMachine::clampAboveTable(std::array<double, 3>& target) const {
  double min_z = table_z_ + table_clearance_;
  if (target[2] < min_z) {
    target[2] = min_z;
  }
}

std::array<float, 6> GraspStateMachine::oneHot() const {
  std::array<float, 6> vec{};
  static const std::array<std::string, 6> kNames = {
      "center_xy", "approach", "descend", "close", "settle", "lift"
  };
  for (size_t i = 0; i < kNames.size(); ++i) {
    if (kNames[i] == state_) {
      vec[i] = 1.0f;
      break;
    }
  }
  return vec;
}

bool GraspStateMachine::isDone() const {
  return state_ == "lift" && state_steps_ > lift_steps_;
}

}  // namespace openarmx_deploy
