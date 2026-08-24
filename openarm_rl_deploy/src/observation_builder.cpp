#include "openarmx_deploy/observation_builder.h"

#include <algorithm>
#include <cstring>

namespace openarmx_deploy {

ObservationBuilder::ObservationBuilder() {
  std::memset(obs_.data(), 0, sizeof(obs_));
}

void ObservationBuilder::updateArmQpos(const std::vector<double>& positions) {
  for (size_t i = 0; i < kArmJoints && i < positions.size(); ++i) {
    obs_[i] = static_cast<float>(positions[i]);
  }
}

void ObservationBuilder::updateGripperQpos(const std::vector<double>& positions) {
  size_t offset = kArmJoints;
  for (size_t i = 0; i < kGripperJoints && i < positions.size(); ++i) {
    obs_[offset + i] = static_cast<float>(positions[i]);
  }
}

void ObservationBuilder::updateObjectPos(double x, double y, double z) {
  size_t offset = kArmJoints + kGripperJoints;
  obs_[offset + 0] = static_cast<float>(x);
  obs_[offset + 1] = static_cast<float>(y);
  obs_[offset + 2] = static_cast<float>(z);
  obj_pos_ = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
}

void ObservationBuilder::updateFingerpadCenter(double x, double y, double z) {
  size_t offset = kArmJoints + kGripperJoints + 3;
  obs_[offset + 0] = static_cast<float>(x);
  obs_[offset + 1] = static_cast<float>(y);
  obs_[offset + 2] = static_cast<float>(z);

  // obj2pad (offset 15-17) = object_pos - fingerpad_center
  size_t obj2pad_offset = kArmJoints + kGripperJoints + 3 + 3;
  obs_[obj2pad_offset + 0] = obj_pos_[0] - static_cast<float>(x);
  obs_[obj2pad_offset + 1] = obj_pos_[1] - static_cast<float>(y);
  obs_[obj2pad_offset + 2] = obj_pos_[2] - static_cast<float>(z);
}

void ObservationBuilder::updateTarget(const std::array<double, 3>& target_pos,
                                      double gripper_target) {
  // offset: arm(7) + grip(2) + obj(3) + pad(3) + obj2pad(3) = 18
  size_t offset = kArmJoints + kGripperJoints + 3 + 3 + 3;
  obs_[offset + 0] = static_cast<float>(target_pos[0]);
  obs_[offset + 1] = static_cast<float>(target_pos[1]);
  obs_[offset + 2] = static_cast<float>(target_pos[2]);
  // gripper_target: offset 21
  obs_[offset + 3] = static_cast<float>(gripper_target);
}

void ObservationBuilder::updateState(int step_count, int max_steps,
                                     const std::array<float, kStateNames>& state_one_hot) {
  // step_fraction: offset 22 (arm+grip+obj+pad+obj2pad+target+grip_target)
  size_t step_offset = kArmJoints + kGripperJoints + 3 + 3 + 3 + 3 + 1;
  obs_[step_offset] = static_cast<float>(step_count) / std::max(1, max_steps);

  // state_one_hot: offset 23
  size_t state_offset = step_offset + 1;
  for (size_t i = 0; i < kStateNames; ++i) {
    obs_[state_offset + i] = state_one_hot[i];
  }
}

std::vector<float> ObservationBuilder::build() const {
  return std::vector<float>(obs_.begin(), obs_.end());
}

}  // namespace openarmx_deploy
