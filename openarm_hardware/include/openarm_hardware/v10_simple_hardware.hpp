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

#include <atomic>
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
  // 以下几个接口属于SystemInterface要求的标准生命周期接口，分别对应硬件的初始化、配置、激活、停用、读取、写入等
  /*
  //插件加载的时候会自动调用生命周期回调
    controller->on_init();      // 初始化
    controller->on_configure(); // 配置
    controller->on_activate();  // 激活
  */
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC      // 是ROS2用于控制符号可见性的宏（对应 __attribute__((visibility("default")))），确保这些方法能被插件系统（pluginlib）识别和调用
  hardware_interface::CallbackReturn on_init(     // 初始化电机
      const hardware_interface::HardwareInfo& info) override;   // override显式声明重载基类的虚函数，避免隐式重载错误，提升代码可读性

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(  // 刷新电机状态，获取电机状态
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()  // 导出状态接口：向 ROS 2 Control 注册硬件的可读取状态，让框架知道从哪里获取每个关节的状态数据
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()  //导出指令接口：向 ROS 2 Control 注册硬件的可写入指令（如关节位置、速度、力矩指令），让框架能下发控制指令。
      override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::return_type prepare_command_mode_switch(    // 重写虚函数，当向服务请求模式切换时，controller_manager会调用这个函数（ros2 control框架生命周期管理的方式），硬件接口可以在这里准备切换（如同步状态到命令，打印日志等），但不执行实际切换逻辑。
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::return_type perform_command_mode_switch(    // 重写虚函数，当向服务请求模式切换时，controller_manager会调用这个函数，硬件接口可以在这里执行实际切换逻辑。
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(   //硬件激活：将硬件从「配置态」切换到「激活态」（如使能电机、归位到零位）
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(  // 硬件去激活：将硬件从「激活态」切回「非激活态」（如禁用电机、停止 CAN 通信），用于安全停机。
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,  // 读取电机状态：周期性从 CAN 总线读取电机状态，写入 pos_states_/vel_states_/tau_states_
                                       const rclcpp::Duration& period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(    // 写入控制指令：周期性将 ROS 2 Control 下发的指令（位置 / 速度 / 力矩）通过 CAN 总线发送给电机执行。
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
    // RS03 + RS03 + RS06 + RS06 + RS00 + RS00 + RS00 + RS00(gripper)
    std::vector<uint8_t> robstride_joint_ids_ = {0x01, 0x02, 0x03, 0x04,
                                                 0x05, 0x06, 0x07};
    std::vector<int> robstride_joint_types_ = {3, 3, 6, 6, 0, 0, 0};
    uint8_t robstride_master_id_ = 0xFD;
    uint8_t robstride_gripper_id_ = 0x08;
    int robstride_gripper_type_ = 0;

  // Gains kp_ 决定「关节有多快能到达目标位置」，kd_ 决定「关节到达目标位置时有多稳」
  std::vector<double> kp_ = {70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0};
  std::vector<double> kd_ = {2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5};

  std::atomic<bool> effort_mode_{false};
  double zero_torque_kd_{0.3};

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
  bool auto_return_to_zero_on_activate_ = false;

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

  // Temporary write inhibit window used after auto homing to avoid abrupt
  // mode handover while controllers are still being activated.
  std::chrono::steady_clock::time_point inhibit_robstride_write_until_ =
      std::chrono::steady_clock::time_point::min();

  // Helper methods
  void return_to_zero();
    void sync_commands_to_current_state();
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
