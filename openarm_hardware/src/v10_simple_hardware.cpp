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

#include "openarm_hardware/v10_simple_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

OpenArm_v10HW::OpenArm_v10HW() = default;

//两个工具函数，核心作用是将以逗号分隔的字符串解析为指定长度的数值列表并返回（分别对应 uint8_t 类型和 int 类型）
//是 ROS 2 Control 硬件接口中解析配置文件参数的关键逻辑，用于处理 robstride_joint_ids、robstride_joint_types 等列表型配置参数。

// 解析 robstride_joint_ids（关节 ID，必须是 0~255 的无符号字节）
std::vector<uint8_t> OpenArm_v10HW::parse_u8_list(const std::string& value,
                                                  size_t expected_size) const {
  std::vector<uint8_t> out;
  std::stringstream ss(value);
  std::string token;

  while (std::getline(ss, token, ',')) {
    token.erase(std::remove_if(token.begin(), token.end(), ::isspace),
                token.end());
    if (token.empty()) {
      continue;
    }
    int parsed = std::stoi(token);
    if (parsed < 0 || parsed > 255) {
      throw std::runtime_error("uint8 list element out of range: " + token);
    }
    out.push_back(static_cast<uint8_t>(parsed));
  }

  if (out.size() != expected_size) {
    throw std::runtime_error("uint8 list size mismatch, expected " +
                             std::to_string(expected_size) + ", got " +
                             std::to_string(out.size()));
  }
  return out;
}

// 解析 robstride_joint_types（关节类型，如 6/3/0 等整数标识）
std::vector<int> OpenArm_v10HW::parse_int_list(const std::string& value,
                                               size_t expected_size) const {
  std::vector<int> out;
  std::stringstream ss(value);
  std::string token;

  while (std::getline(ss, token, ',')) {
    token.erase(std::remove_if(token.begin(), token.end(), ::isspace),
                token.end());
    if (token.empty()) {
      continue;
    }
    out.push_back(std::stoi(token));
  }

  if (out.size() != expected_size) {
    throw std::runtime_error("int list size mismatch, expected " +
                             std::to_string(expected_size) + ", got " +
                             std::to_string(out.size()));
  }
  return out;
}

// 这个函数解析的“源”是 ros2_control 在 URDF 里定义的 hardware 参数表，不是直接读某个 YAML 或 launch 参数
bool OpenArm_v10HW::parse_config(const hardware_interface::HardwareInfo& info) {
  auto parse_bool = [](const std::string& value, bool default_value) {
    if (value.empty()) {
      return default_value;
    }
    std::string normalized = value;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   ::tolower);
    return normalized == "true" || normalized == "1" || normalized == "yes";
  };

  // Parse CAN interface (default: can0)
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  // Parse arm prefix (default: empty for single arm, "left_" or "right_" for
  // bimanual)
  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  // Parse motor backend
  it = info.hardware_parameters.find("motor_backend");
  motor_backend_str_ =
      (it != info.hardware_parameters.end()) ? it->second : "damiao";
  std::transform(motor_backend_str_.begin(), motor_backend_str_.end(),
                 motor_backend_str_.begin(), ::tolower);
  if (motor_backend_str_ == "damiao") {
    motor_backend_ = MotorBackend::kDamiao;
  } else if (motor_backend_str_ == "robstride") {
    motor_backend_ = MotorBackend::kRobStride;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Unsupported motor_backend '%s'. Use damiao or robstride.",
                 motor_backend_str_.c_str());
    return false;
  }

  // Parse gripper enable (default: true for V10)
  it = info.hardware_parameters.find("hand");
  hand_ = (it == info.hardware_parameters.end())
              ? true
              : parse_bool(it->second, true);

  // Parse CAN-FD enable (default: true for V10)
  it = info.hardware_parameters.find("can_fd");
  can_fd_ = (it == info.hardware_parameters.end())
                ? true
                : parse_bool(it->second, true);

  // Parse control gains
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    it = info.hardware_parameters.find("kp" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kp_[i - 1] = std::stod(it->second);
    }
    it = info.hardware_parameters.find("kd" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kd_[i - 1] = std::stod(it->second);
    }
  }

  if (motor_backend_ == MotorBackend::kRobStride) {
    try {
      it = info.hardware_parameters.find("robstride_master_id");
      if (it != info.hardware_parameters.end()) {
        int master = std::stoi(it->second);
        if (master < 0 || master > 255) {
          throw std::runtime_error("robstride_master_id out of range");
        }
        robstride_master_id_ = static_cast<uint8_t>(master);
      }

      it = info.hardware_parameters.find("robstride_joint_ids");
      if (it != info.hardware_parameters.end()) {
        robstride_joint_ids_ = parse_u8_list(it->second, ARM_DOF);
      }

      it = info.hardware_parameters.find("robstride_joint_types");
      if (it != info.hardware_parameters.end()) {
        robstride_joint_types_ = parse_int_list(it->second, ARM_DOF);
      }

      for (size_t i = 0; i < robstride_joint_types_.size(); ++i) {
        if (robstride_joint_types_[i] < 0 || robstride_joint_types_[i] > 6) {
          throw std::runtime_error("robstride_joint_types contains invalid "
                                   "actuator type at index " +
                                   std::to_string(i));
        }
      }

      it = info.hardware_parameters.find("robstride_gripper_id");
      if (it != info.hardware_parameters.end()) {
        int gripper_id = std::stoi(it->second);
        if (gripper_id < 0 || gripper_id > 255) {
          throw std::runtime_error("robstride_gripper_id out of range");
        }
        robstride_gripper_id_ = static_cast<uint8_t>(gripper_id);
      }

      it = info.hardware_parameters.find("robstride_gripper_type");
      if (it != info.hardware_parameters.end()) {
        robstride_gripper_type_ = std::stoi(it->second);
        if (robstride_gripper_type_ < 0 || robstride_gripper_type_ > 6) {
          throw std::runtime_error("robstride_gripper_type out of range");
        }
      }
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                   "Failed to parse RobStride configuration: %s", ex.what());
      return false;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Configuration: backend=%s, CAN=%s, arm_prefix=%s, hand=%s, "
              "can_fd=%s",
              motor_backend_str_.c_str(), can_interface_.c_str(),
              arm_prefix_.c_str(), hand_ ? "enabled" : "disabled",
              can_fd_ ? "enabled" : "disabled");

  if (motor_backend_ == MotorBackend::kRobStride) {
    RCLCPP_INFO(
        rclcpp::get_logger("OpenArm_v10HW"),
        "RobStride mapping loaded: master_id=%u, joint_ids=%u,%u,%u,%u,%u,%u,%u, "
        "joint_types=%d,%d,%d,%d,%d,%d,%d, gripper_id=%u, gripper_type=%d",
        static_cast<unsigned>(robstride_master_id_),
        static_cast<unsigned>(robstride_joint_ids_[0]),
        static_cast<unsigned>(robstride_joint_ids_[1]),
        static_cast<unsigned>(robstride_joint_ids_[2]),
        static_cast<unsigned>(robstride_joint_ids_[3]),
        static_cast<unsigned>(robstride_joint_ids_[4]),
        static_cast<unsigned>(robstride_joint_ids_[5]),
        static_cast<unsigned>(robstride_joint_ids_[6]),
        robstride_joint_types_[0], robstride_joint_types_[1],
        robstride_joint_types_[2], robstride_joint_types_[3],
        robstride_joint_types_[4], robstride_joint_types_[5],
        robstride_joint_types_[6], static_cast<unsigned>(robstride_gripper_id_),
        robstride_gripper_type_);
  }

  return true;
}

void OpenArm_v10HW::generate_joint_names() {
  joint_names_.clear();
  // TODO: read from urdf properly and sort in the future.
  // Currently, the joint names are hardcoded for order consistency to align
  // with hardware. Generate arm joint names: openarm_{arm_prefix}joint{N}
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    std::string joint_name =
        "openarm_" + arm_prefix_ + "joint" + std::to_string(i);
    joint_names_.push_back(joint_name);
  }

  // Generate gripper joint name if enabled
  if (hand_) {
    std::string gripper_joint_name = "openarm_" + arm_prefix_ + "finger_joint1";
    joint_names_.push_back(gripper_joint_name);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Gripper joint NOT added because hand_=false");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Generated %zu joint names for arm prefix '%s'",
              joint_names_.size(), arm_prefix_.c_str());
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Parse configuration
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Generate joint names based on arm prefix
  generate_joint_names();

  // Validate joint count (7 arm joints + optional gripper)
  size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0);
  if (joint_names_.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Generated %zu joint names, expected %zu", joint_names_.size(),
                 expected_joints);
    return CallbackReturn::ERROR;
  }

  // Initialize state and command vectors based on generated joint count
  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  if (motor_backend_ == MotorBackend::kDamiao) {
    if (!init_damiao_backend()) {
      return CallbackReturn::ERROR;
    }
  } else {
    if (!init_robstride_backend()) {
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "OpenArm V10 hardware initialized successfully with backend '%s'",
              motor_backend_str_.c_str());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (motor_backend_ == MotorBackend::kDamiao) {
    return configure_damiao_backend();
  }
  return configure_robstride_backend();
}


/*
与 ROS 2 Control 框架的交互流程
框架启动时调用 export_state_interfaces()，获取所有状态接口的「名称 - 类型 - 地址」映射；
框架周期性调用 read() 方法，硬件接口从 CAN 总线读取电机状态，写入 pos_states_/vel_states_/tau_states_；
框架通过 export_state_interfaces() 注册的内存地址，直接读取这些向量的值，供控制器（如 MoveIt 2、关节轨迹控制器）使用。
*/
std::vector<hardware_interface::StateInterface>   // StateInterface 是 ROS 2 Control 定义的「状态接口」类，用于描述「硬件状态数据的名称 + 数据类型 + 内存地址」
OpenArm_v10HW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {    // 循环遍历每个关节，为每个关节生成「位置、速度、力矩」三种状态接口，并将它们注册到ROS2 Control框架中。这样，控制器就可以通过这些接口获取每个关节的当前状态数据。
    state_interfaces.emplace_back(hardware_interface::StateInterface(   // emplace_back：直接在向量中构造 StateInterface 对象（效率高于 push_back）
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArm_v10HW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // TODO: consider exposing only needed interfaces to avoid undefined behavior.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (motor_backend_ == MotorBackend::kDamiao) {
    return activate_damiao_backend();
  }
  return activate_robstride_backend();
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (motor_backend_ == MotorBackend::kDamiao) {
    return deactivate_damiao_backend();
  }
  return deactivate_robstride_backend();
}

hardware_interface::return_type OpenArm_v10HW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (motor_backend_ == MotorBackend::kDamiao) {
    return read_damiao_backend();
  }
  return read_robstride_backend();
}

hardware_interface::return_type OpenArm_v10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (motor_backend_ == MotorBackend::kDamiao) {
    return write_damiao_backend();
  }
  return write_robstride_backend();
}

void OpenArm_v10HW::return_to_zero() {
  if (motor_backend_ == MotorBackend::kDamiao) {
    return_to_zero_damiao();
    return;
  }
  return_to_zero_robstride();
}

bool OpenArm_v10HW::init_damiao_backend() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Initializing Damiao OpenArm backend on %s with CAN-FD %s...",
              can_interface_.c_str(), can_fd_ ? "enabled" : "disabled");

  openarm_ =
      std::make_unique<openarm::can::socket::OpenArm>(can_interface_, can_fd_);

  openarm_->init_arm_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS,
                            DEFAULT_RECV_CAN_IDS);

  if (hand_) {
    openarm_->init_gripper_motor(DEFAULT_GRIPPER_MOTOR_TYPE,
                                 DEFAULT_GRIPPER_SEND_CAN_ID,
                                 DEFAULT_GRIPPER_RECV_CAN_ID);
  }

  return true;
}

bool OpenArm_v10HW::init_robstride_backend() {
  try {
    robstride_arm_motors_.clear();
    robstride_arm_motors_.reserve(ARM_DOF);

    for (size_t i = 0; i < ARM_DOF; ++i) {
      robstride_arm_motors_.emplace_back(std::make_unique<RobStrideMotor>(
          can_interface_, robstride_master_id_, robstride_joint_ids_[i],
          robstride_joint_types_[i]));
    }

    if (hand_) {
      robstride_gripper_motor_ = std::make_unique<RobStrideMotor>(
          can_interface_, robstride_master_id_, robstride_gripper_id_,
          robstride_gripper_type_);
    }

    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Initialized RobStride backend on %s for %zu joints",
                can_interface_.c_str(), robstride_arm_motors_.size());
    return true;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "RobStride backend initialization failed: %s", ex.what());
    return false;
  }
}

hardware_interface::CallbackReturn OpenArm_v10HW::configure_damiao_backend() {
  openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::configure_robstride_backend() {
  for (auto& motor : robstride_arm_motors_) {
    motor->receive_status_frame(0.01);
  }
  if (hand_ && robstride_gripper_motor_) {
    robstride_gripper_motor_->receive_status_frame(0.01);
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::activate_damiao_backend() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Activating OpenArm V10 with Damiao backend...");
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
  openarm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  return_to_zero();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "OpenArm V10 activated (Damiao backend)");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::activate_robstride_backend() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Activating OpenArm V10 with RobStride backend...");

  for (auto& motor : robstride_arm_motors_) {
    motor->Get_RobStrite_Motor_parameter(0x7005);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    motor->enable_motor();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (hand_ && robstride_gripper_motor_) {
    robstride_gripper_motor_->Get_RobStrite_Motor_parameter(0x7005);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    robstride_gripper_motor_->enable_motor();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return_to_zero();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "OpenArm V10 activated (RobStride backend)");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::deactivate_damiao_backend() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Deactivating OpenArm V10 (Damiao backend)...");

  openarm_->disable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::deactivate_robstride_backend() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Deactivating OpenArm V10 (RobStride backend)...");

  for (auto& motor : robstride_arm_motors_) {
    motor->Disenable_Motor(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (hand_ && robstride_gripper_motor_) {
    robstride_gripper_motor_->Disenable_Motor(0);
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArm_v10HW::read_damiao_backend() {
  openarm_->refresh_all();
  openarm_->recv_all();

  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    pos_states_[i] = arm_motors[i].get_position();
    vel_states_[i] = arm_motors[i].get_velocity();
    tau_states_[i] = arm_motors[i].get_torque();
  }

  if (hand_ && joint_names_.size() > ARM_DOF) {
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      double motor_pos = gripper_motors[0].get_position();
      pos_states_[ARM_DOF] = motor_radians_to_joint(motor_pos);
      vel_states_[ARM_DOF] = 0;
      tau_states_[ARM_DOF] = 0;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10HW::read_robstride_backend() {
  for (size_t i = 0; i < ARM_DOF && i < robstride_arm_motors_.size(); ++i) {
    robstride_arm_motors_[i]->receive_status_frame(0.002);
    pos_states_[i] = robstride_arm_motors_[i]->position_;
    vel_states_[i] = robstride_arm_motors_[i]->velocity_;
    tau_states_[i] = robstride_arm_motors_[i]->torque_;
  }

  if (hand_ && robstride_gripper_motor_ && joint_names_.size() > ARM_DOF) {
    robstride_gripper_motor_->receive_status_frame(0.002);
    pos_states_[ARM_DOF] =
        motor_radians_to_joint(robstride_gripper_motor_->position_);
    vel_states_[ARM_DOF] = robstride_gripper_motor_->velocity_;
    tau_states_[ARM_DOF] = robstride_gripper_motor_->torque_;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10HW::write_damiao_backend() {
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    arm_params.push_back(
        {kp_[i], kd_[i], pos_commands_[i], vel_commands_[i], tau_commands_[i]});
  }
  openarm_->get_arm().mit_control_all(arm_params);

  if (hand_ && joint_names_.size() > ARM_DOF) {
    double motor_command = joint_to_motor_radians(pos_commands_[ARM_DOF]);
    openarm_->get_gripper().mit_control_all(
        {{GRIPPER_KP, GRIPPER_KD, motor_command, 0, 0}});
  }

  openarm_->recv_all(1000);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10HW::write_robstride_backend() {
  for (size_t i = 0; i < ARM_DOF && i < robstride_arm_motors_.size(); ++i) {
    robstride_arm_motors_[i]->send_motion_command(
        static_cast<float>(tau_commands_[i]),
        static_cast<float>(pos_commands_[i]),
        static_cast<float>(vel_commands_[i]), static_cast<float>(kp_[i]),
        static_cast<float>(kd_[i]));
  }

  if (hand_ && robstride_gripper_motor_ && joint_names_.size() > ARM_DOF) {
    const float gripper_motor_command =
        static_cast<float>(joint_to_motor_radians(pos_commands_[ARM_DOF]));
    robstride_gripper_motor_->send_motion_command(
        0.0f, gripper_motor_command, 0.0f, static_cast<float>(GRIPPER_KP),
        static_cast<float>(GRIPPER_KD));
  }

  return hardware_interface::return_type::OK;
}

void OpenArm_v10HW::return_to_zero_damiao() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Returning to zero position (Damiao backend)...");

  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    arm_params.push_back({kp_[i], kd_[i], 0.0, 0.0, 0.0});
  }
  openarm_->get_arm().mit_control_all(arm_params);

  if (hand_) {
    openarm_->get_gripper().mit_control_all(
        {{GRIPPER_KP, GRIPPER_KD, GRIPPER_JOINT_0_POSITION, 0.0, 0.0}});
  }
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  openarm_->recv_all();
}

void OpenArm_v10HW::return_to_zero_robstride() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Returning to zero position (RobStride backend)...");

  for (size_t i = 0; i < ARM_DOF && i < robstride_arm_motors_.size(); ++i) {
    robstride_arm_motors_[i]->send_motion_command(
        0.0f, 0.0f, 0.0f, static_cast<float>(kp_[i]), static_cast<float>(kd_[i]));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (hand_ && robstride_gripper_motor_) {
    robstride_gripper_motor_->send_motion_command(
        0.0f, static_cast<float>(joint_to_motor_radians(0.0)), 0.0f,
        static_cast<float>(GRIPPER_KP), static_cast<float>(GRIPPER_KD));
  }
}

// Gripper mapping helper functions
double OpenArm_v10HW::joint_to_motor_radians(double joint_value) {
  // Joint 0=closed -> motor 0 rad, Joint 0.044=open -> motor -1.0472 rad
  return (joint_value / GRIPPER_JOINT_0_POSITION) *
         GRIPPER_MOTOR_1_RADIANS;  // Scale from 0-0.044 to 0 to -1.0472
}

double OpenArm_v10HW::motor_radians_to_joint(double motor_radians) {
  // Motor 0 rad=closed -> joint 0, Motor -1.0472 rad=open -> joint 0.044
  return GRIPPER_JOINT_0_POSITION *
         (motor_radians /
          GRIPPER_MOTOR_1_RADIANS);  // Scale from 0 to -1.0472 to 0-0.044
}

}  // namespace openarm_hardware

// 核心注册逻辑，用于将自定义的 OpenArm_v10HW 硬件接口类注册为 ROS2 Control 框架可识别的插件，使得框架能通过插件机制动态加载并实例化该硬件接口
// PLUGINLIB_EXPORT_CLASS第一个参数是自定义子类，第二个是基类
// 借助 pluginlib 实现插件化扩展
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10HW,
                       hardware_interface::SystemInterface)
