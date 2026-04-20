// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <motor_ros2/motor_cfg.h>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "openarm_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace openarm_hardware {

/**
 * @brief Simplified OpenArm V10 Hardware Interface
 *
 * This is a simplified version that uses the OpenArm CAN API directly,
 * following the pattern from full_arm.cpp example. Much simpler than
 * the original implementation.
 */
class OpenArm_v10HW : public hardware_interface::SystemInterface {
 public:
  OpenArm_v10HW();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
    enum class MotorBackend {
        kDamiao,
        kRobStride,
    };

  // V10 default configuration
  static constexpr size_t ARM_DOF = 7;

    // Default Damiao motor configuration for V10
  const std::vector<openarm::damiao_motor::MotorType> DEFAULT_MOTOR_TYPES = {
      openarm::damiao_motor::MotorType::DM8009,  // Joint 1
      openarm::damiao_motor::MotorType::DM8009,  // Joint 2
      openarm::damiao_motor::MotorType::DM4340,  // Joint 3
      openarm::damiao_motor::MotorType::DM4340,  // Joint 4
      openarm::damiao_motor::MotorType::DM4310,  // Joint 5
      openarm::damiao_motor::MotorType::DM4310,  // Joint 6
      openarm::damiao_motor::MotorType::DM4310   // Joint 7
  };

  const std::vector<uint32_t> DEFAULT_SEND_CAN_IDS = {0x01, 0x02, 0x03, 0x04,
                                                      0x05, 0x06, 0x07};
  const std::vector<uint32_t> DEFAULT_RECV_CAN_IDS = {0x11, 0x12, 0x13, 0x14,
                                                      0x15, 0x16, 0x17};

  const openarm::damiao_motor::MotorType DEFAULT_GRIPPER_MOTOR_TYPE =
      openarm::damiao_motor::MotorType::DM4310;
  const uint32_t DEFAULT_GRIPPER_SEND_CAN_ID = 0x08;
  const uint32_t DEFAULT_GRIPPER_RECV_CAN_ID = 0x18;

    // Default RobStride motor configuration for V10:
    // RS06 + RS06 + RS03 + RS00 + RS00 + RS00 + RS00 + RS00(gripper)
    std::vector<uint8_t> robstride_joint_ids_ = {0x01, 0x02, 0x03, 0x04,
                                                                                             0x05, 0x06, 0x07};
    std::vector<int> robstride_joint_types_ = {6, 6, 3, 0, 0, 0, 0};
    uint8_t robstride_master_id_ = 0xFD;
    uint8_t robstride_gripper_id_ = 0x08;
    int robstride_gripper_type_ = 0;

  // Gains
  std::vector<double> kp_ = {70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0};
  std::vector<double> kd_ = {2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5};

  const double GRIPPER_JOINT_0_POSITION = 0.044;
  const double GRIPPER_JOINT_1_POSITION = 0.0;
  const double GRIPPER_MOTOR_0_RADIANS = 0.0;
  const double GRIPPER_MOTOR_1_RADIANS = -1.0472;
  const double GRIPPER_KP = 5.0;
  const double GRIPPER_KD = 0.1;

  // Configuration
  std::string can_interface_;
  std::string arm_prefix_;
    std::string motor_backend_str_;
    MotorBackend motor_backend_ = MotorBackend::kDamiao;
  bool hand_;
  bool can_fd_;

    // Damiao backend instance
  std::unique_ptr<openarm::can::socket::OpenArm> openarm_;

    // RobStride backend instances
    std::vector<std::unique_ptr<RobStrideMotor>> robstride_arm_motors_;
    std::unique_ptr<RobStrideMotor> robstride_gripper_motor_;

  // Generated joint names for this arm instance
  std::vector<std::string> joint_names_;

  // ROS2 control state and command vectors
  std::vector<double> pos_commands_;
  std::vector<double> vel_commands_;
  std::vector<double> tau_commands_;
  std::vector<double> pos_states_;
  std::vector<double> vel_states_;
  std::vector<double> tau_states_;

  // Helper methods
  void return_to_zero();
    void return_to_zero_damiao();
    void return_to_zero_robstride();

    bool init_damiao_backend();
    bool init_robstride_backend();

    hardware_interface::CallbackReturn configure_damiao_backend();
    hardware_interface::CallbackReturn configure_robstride_backend();

    hardware_interface::CallbackReturn activate_damiao_backend();
    hardware_interface::CallbackReturn activate_robstride_backend();

    hardware_interface::CallbackReturn deactivate_damiao_backend();
    hardware_interface::CallbackReturn deactivate_robstride_backend();

    hardware_interface::return_type read_damiao_backend();
    hardware_interface::return_type read_robstride_backend();

    hardware_interface::return_type write_damiao_backend();
    hardware_interface::return_type write_robstride_backend();

  bool parse_config(const hardware_interface::HardwareInfo& info);
  void generate_joint_names();
    std::vector<uint8_t> parse_u8_list(const std::string& value,
                                                                         size_t expected_size) const;
    std::vector<int> parse_int_list(const std::string& value,
                                                                    size_t expected_size) const;

  // Gripper mapping functions
  double joint_to_motor_radians(double joint_value);
  double motor_radians_to_joint(double motor_radians);
};

}  // namespace openarm_hardware
