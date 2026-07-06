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
#include <cmath>
#include <limits>
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

  // Parse arm type to select version-specific limit tables.
  it = info.hardware_parameters.find("arm_type");
  arm_type_ = (it != info.hardware_parameters.end()) ? it->second : "v10";
  std::transform(arm_type_.begin(), arm_type_.end(), arm_type_.begin(), ::tolower);

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

  // Safety behavior on activate (default: do not auto return-to-zero)
  it = info.hardware_parameters.find("auto_return_to_zero_on_activate");
  auto_return_to_zero_on_activate_ =
      (it == info.hardware_parameters.end())
          ? false
          : parse_bool(it->second, false);

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

  it = info.hardware_parameters.find("zero_torque_kd");
  if (it != info.hardware_parameters.end()) {
    zero_torque_kd_ = std::stod(it->second);
    zero_torque_kd_ = std::clamp(zero_torque_kd_, 0.0, 5.0);
  }

  it = info.hardware_parameters.find("limit_margin");
  if (it != info.hardware_parameters.end()) {
    limit_margin_ = std::max(0.0, std::stod(it->second));
  }
  it = info.hardware_parameters.find("limit_stop_margin");
  if (it != info.hardware_parameters.end()) {
    limit_stop_margin_ = std::max(0.0, std::stod(it->second));
  }
  it = info.hardware_parameters.find("limit_decel_factor");
  if (it != info.hardware_parameters.end()) {
    limit_decel_factor_ = std::stod(it->second);
    limit_decel_factor_ = std::clamp(limit_decel_factor_, 0.0, 1.0);
  }

  if (limit_stop_margin_ > limit_margin_) {
    limit_stop_margin_ = limit_margin_;
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
              "can_fd=%s, auto_return_to_zero_on_activate=%s",
              motor_backend_str_.c_str(), can_interface_.c_str(),
              arm_prefix_.c_str(), hand_ ? "enabled" : "disabled",
              can_fd_ ? "enabled" : "disabled",
              auto_return_to_zero_on_activate_ ? "true" : "false");

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Limit protection: margin=%.3f, stop_margin=%.3f, decel_factor=%.2f",
              limit_margin_, limit_stop_margin_, limit_decel_factor_);

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

std::array<std::array<double, 2>, OpenArm_v10HW::ARM_DOF>
OpenArm_v10HW::compute_arm_limits() const {
  if (arm_type_ == "v11") {
    std::array<std::array<double, 2>, ARM_DOF> limits = {{
        {{-1.65, 3.21}},   // joint1
        {{-2.89, 0.36}},   // joint2
        {{-3.0, 0.13}},    // joint3
        {{-0.28, 1.63}},   // joint4
        {{-2.72, -0.14}},  // joint5
        {{-0.52, 0.52}},   // joint6
        {{-1.29, 1.29}}    // joint7
    }};

    if (arm_prefix_.find("right_") != std::string::npos) {
      limits[0][0] = -3.23;
      limits[0][1] = 1.63;
      limits[1][0] = -0.35;
      limits[1][1] = 2.89;
      limits[2][0] = -3.12;
      limits[2][1] = 0.0;
      limits[3][0] = -0.24;
      limits[3][1] = 1.68;
      limits[4][0] = -2.7;
      limits[4][1] = -0.12;
    }

    return limits;
  }

  std::array<std::array<double, 2>, ARM_DOF> limits = {{
      {{-1.396263, 3.490659}},  // joint1
      {{-1.745329, 1.745329}},  // joint2
      {{-1.570796, 1.570796}},  // joint3
      {{0.0, 2.443461}},        // joint4
      {{-1.570796, 1.570796}},  // joint5
      {{-0.785398, 0.785398}},  // joint6
      {{-1.570796, 1.570796}}   // joint7
  }};

  if (arm_prefix_.find("right_") != std::string::npos) {
    limits[1][0] = -0.174533;  // joint2: -1.745 + M_PI/2
    limits[1][1] = 3.31613;    // joint2: 1.745 + M_PI/2
  } else if (arm_prefix_.find("left_") != std::string::npos) {
    limits[0][0] = -3.49066;   // joint1: -1.396 - 2.094
    limits[0][1] = 1.39626;    // joint1: 3.490 - 2.094
    limits[1][0] = -3.31613;   // joint2: -1.745 - M_PI/2 (reflect=-1)
    limits[1][1] = 0.174533;   // joint2: 1.745 - M_PI/2 (reflect=-1)
  }

  return limits;
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

//命令接口把内存暴露给控制器，控制器可以直接写入这些内存量的值，从而实现对硬件的控制
//把 pos_commands_ / vel_commands_ / tau_commands_ 的地址注册给 controller_manager，控制器写的就是这些指针指向的内存
//所以通过ros2 action send_goal 发布控制指令，控制器会根据指令更新 pos_commands_ / vel_commands_ / tau_commands_ 中的值，从而实现对硬件的控制
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

hardware_interface::return_type OpenArm_v10HW::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& /*stop_interfaces*/) {
  bool wants_effort = false;
  for (const auto& iface : start_interfaces) {
    if (iface.find("effort") != std::string::npos) {
      wants_effort = true;
      break;
    }
  }

  if (wants_effort) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Preparing switch to effort (zero-torque) mode");
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10HW::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  bool starting_effort = false;
  bool stopping_effort = false;

  for (const auto& iface : start_interfaces) {
    if (iface.find("effort") != std::string::npos) {
      starting_effort = true;
      break;
    }
  }
  for (const auto& iface : stop_interfaces) {
    if (iface.find("effort") != std::string::npos) {
      stopping_effort = true;
      break;
    }
  }

  if (starting_effort && !effort_mode_) {
    effort_mode_ = true;
    sync_commands_to_current_state();
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "Switched to effort mode (Kp=0, Kd=%.3f)", zero_torque_kd_);
  }

  if (stopping_effort && effort_mode_) {
    effort_mode_ = false;
    sync_commands_to_current_state();
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Switched to position mode");
  }

  return hardware_interface::return_type::OK;
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

  // ---- 硬件级绝对防撞兜底: 动态适配不同机械臂配置(单臂/左/右) ----
  // 限制所有关节位置在安全范围内, 硬件和rviz中的可视化表现一致
  // ！！！真机上机前一定需要先小浮动测试，观察是否和仿真环境的运动方向一致
  const auto arm_limits = compute_arm_limits();

  for (size_t i = 0; i < ARM_DOF && i < pos_commands_.size(); ++i) {
    pos_commands_[i] = std::clamp(pos_commands_[i], arm_limits[i][0], arm_limits[i][1]);
  }
  
  if (hand_ && pos_commands_.size() > ARM_DOF) {
    pos_commands_[ARM_DOF] = std::clamp(pos_commands_[ARM_DOF], 0.0, 0.044);
  }
  // -----------------------------------------------------------

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

void OpenArm_v10HW::sync_commands_to_current_state() {    // 将当前状态同步到命令向量，避免模式切换时的突变
  for (size_t i = 0; i < pos_commands_.size() && i < pos_states_.size(); ++i) {
    pos_commands_[i] = pos_states_[i];
    vel_commands_[i] = 0.0;
    tau_commands_[i] = 0.0;
  }
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

  (void)read_damiao_backend();

  if (auto_return_to_zero_on_activate_) {
    return_to_zero();
    std::fill(pos_commands_.begin(), pos_commands_.end(), 0.0);
    std::fill(vel_commands_.begin(), vel_commands_.end(), 0.0);
    std::fill(tau_commands_.begin(), tau_commands_.end(), 0.0);
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "auto_return_to_zero_on_activate=true: commanded zero position "
                "during activation.");
  } else {
    sync_commands_to_current_state();
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "auto_return_to_zero_on_activate=false: holding current "
                "position at activation.");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "OpenArm V10 activated (Damiao backend)");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::activate_robstride_backend() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Activating OpenArm V10 with RobStride backend...");

  auto wait_until_target_reached = [](RobStrideMotor& motor, float target,
                                      float tolerance_rad = 0.01f,
                                      int timeout_ms = 12000,
                                      int poll_interval_ms = 2) {
    constexpr uint16_t kMechPosIndex = 0x7019;
    motor.drw.mechPos.data = std::numeric_limits<float>::quiet_NaN();

    const auto start = std::chrono::steady_clock::now();
    while (true) {
      // PP mode may not stream status continuously, so actively poll mechPos.
      if (!motor.receive_status_frame(0.05, false)) {
        motor.Get_RobStrite_Motor_parameter(kMechPosIndex);
      }

      float feedback_position = motor.position_;
      if (std::isfinite(motor.drw.mechPos.data)) {
        feedback_position = motor.drw.mechPos.data;
      }

      if (std::isfinite(feedback_position) &&
          std::fabs(feedback_position - target) <= tolerance_rad) {
        return true;
      }

      const auto now = std::chrono::steady_clock::now();
      const auto elapsed_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - start)
              .count();
      if (elapsed_ms >= timeout_ms) {
        return false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
    }
  };

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

  (void)read_robstride_backend();

  if (auto_return_to_zero_on_activate_) {
    return_to_zero();

    constexpr float kHomeTarget = 0.0f;
    for (size_t i = 0; i < ARM_DOF && i < robstride_arm_motors_.size(); ++i) {
      const bool reached =
          wait_until_target_reached(*robstride_arm_motors_[i], kHomeTarget);
      if (!reached) {
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                    "joint%zu did not reach target within timeout during "
                    "activation return-to-zero.",
                    i + 1);
      }
    }

    if (hand_ && robstride_gripper_motor_) {
      const float gripper_target =
          static_cast<float>(joint_to_motor_radians(0.0));
      const bool reached =
          wait_until_target_reached(*robstride_gripper_motor_, gripper_target);
      if (!reached) {
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                    "Gripper did not reach target within timeout during "
                    "activation return-to-zero.");
      }
    }

    // Safety: initialize commands to homing targets directly instead of relying
    // on a best-effort read that may contain stale values when CAN feedback
    // times out intermittently.
    for (size_t i = 0; i < ARM_DOF && i < pos_commands_.size(); ++i) {
      pos_states_[i] = kHomeTarget;
      vel_states_[i] = 0.0;
      tau_states_[i] = 0.0;

      pos_commands_[i] = kHomeTarget;
      vel_commands_[i] = 0.0;
      tau_commands_[i] = 0.0;
    }

    if (hand_ && joint_names_.size() > ARM_DOF) {
      pos_states_[ARM_DOF] = 0.0;
      vel_states_[ARM_DOF] = 0.0;
      tau_states_[ARM_DOF] = 0.0;

      pos_commands_[ARM_DOF] = 0.0;
      vel_commands_[ARM_DOF] = 0.0;
      tau_commands_[ARM_DOF] = 0.0;
    }

    inhibit_robstride_write_until_ =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(800);

    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "auto_return_to_zero_on_activate=true: performed PP return-to-zero "
                "with completion wait, then latched command/state to homing target.");
  } else {
    inhibit_robstride_write_until_ = std::chrono::steady_clock::time_point::min();
    sync_commands_to_current_state();
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "auto_return_to_zero_on_activate=false: holding current "
                "position at activation.");
  }

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
    const bool received = robstride_arm_motors_[i]->receive_status_frame(0.002, false);
    if (!received) {
      // Keep publishing cached values so state interfaces do not freeze at
      // initialization values when frames are intermittently missed.
    }
    pos_states_[i] = robstride_arm_motors_[i]->position_;
    vel_states_[i] = robstride_arm_motors_[i]->velocity_;
    tau_states_[i] = robstride_arm_motors_[i]->torque_;
  }

  if (hand_ && robstride_gripper_motor_ && joint_names_.size() > ARM_DOF) {
    const bool received =
        robstride_gripper_motor_->receive_status_frame(0.002, false);
    if (!received) {
      // return hardware_interface::return_type::OK;
      // Keep publishing cached values so gripper state does not freeze when
      // feedback frames are intermittent.
    }
    pos_states_[ARM_DOF] =
        motor_radians_to_joint(robstride_gripper_motor_->position_);
    vel_states_[ARM_DOF] = robstride_gripper_motor_->velocity_;
    tau_states_[ARM_DOF] = robstride_gripper_motor_->torque_;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10HW::write_damiao_backend() {
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  const bool effort_mode = effort_mode_.load();
  const auto arm_limits = compute_arm_limits();
  auto apply_effort_limit = [&](size_t idx, double pos, double tau) {
    if (!effort_mode) {
      return tau;
    }

    const double lower = arm_limits[idx][0];
    const double upper = arm_limits[idx][1];
    if (pos <= lower + limit_stop_margin_ && tau < 0.0) {
      return 0.0;
    }
    if (pos >= upper - limit_stop_margin_ && tau > 0.0) {
      return 0.0;
    }
    if (pos <= lower + limit_margin_ && tau < 0.0) {
      return tau * limit_decel_factor_;
    }
    if (pos >= upper - limit_margin_ && tau > 0.0) {
      return tau * limit_decel_factor_;
    }
    return tau;
  };

  for (size_t i = 0; i < ARM_DOF; ++i) {
    const double cmd_kp = effort_mode ? 0.0 : kp_[i];
    const double cmd_kd = effort_mode ? zero_torque_kd_ : kd_[i];
    const double cmd_pos = effort_mode ? pos_states_[i] : pos_commands_[i];
    const double cmd_vel = effort_mode ? 0.0 : vel_commands_[i];
    double cmd_tau = tau_commands_[i];
    cmd_tau = apply_effort_limit(i, pos_states_[i], cmd_tau);
    arm_params.push_back({cmd_kp, cmd_kd, cmd_pos, cmd_vel, cmd_tau});
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
  if (std::chrono::steady_clock::now() < inhibit_robstride_write_until_) {
    return hardware_interface::return_type::OK;
  }

  const bool effort_mode = effort_mode_.load();
  const auto arm_limits = compute_arm_limits();
  auto apply_effort_limit = [&](size_t idx, double pos, float tau) {
    if (!effort_mode) {
      return tau;
    }

    const double lower = arm_limits[idx][0];
    const double upper = arm_limits[idx][1];
    if (pos <= lower + limit_stop_margin_ && tau < 0.0f) {
      return 0.0f;
    }
    if (pos >= upper - limit_stop_margin_ && tau > 0.0f) {
      return 0.0f;
    }
    if (pos <= lower + limit_margin_ && tau < 0.0f) {
      return tau * static_cast<float>(limit_decel_factor_);
    }
    if (pos >= upper - limit_margin_ && tau > 0.0f) {
      return tau * static_cast<float>(limit_decel_factor_);
    }
    return tau;
  };

  for (size_t i = 0; i < ARM_DOF && i < robstride_arm_motors_.size(); ++i) {
    const float cmd_kp = effort_mode ? 0.0f : static_cast<float>(kp_[i]);
    const float cmd_kd = effort_mode ? static_cast<float>(zero_torque_kd_)
                                     : static_cast<float>(kd_[i]);
    const float cmd_pos = effort_mode ? static_cast<float>(pos_states_[i])
                                      : static_cast<float>(pos_commands_[i]);
    const float cmd_vel = effort_mode ? 0.0f : static_cast<float>(vel_commands_[i]);
    float cmd_tau = static_cast<float>(tau_commands_[i]);
    cmd_tau = apply_effort_limit(i, pos_states_[i], cmd_tau);
    robstride_arm_motors_[i]->send_motion_command(
        cmd_tau,
        cmd_pos,
        cmd_vel,
        cmd_kp,
        cmd_kd);
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

  constexpr float kReturnZeroSpeed = 1.0f;
  constexpr float kReturnZeroAcceleration = 5.0f;
  constexpr float kReturnZeroTarget = 0.0f;

  for (size_t i = 0; i < ARM_DOF && i < robstride_arm_motors_.size(); ++i) {
    robstride_arm_motors_[i]->RobStrite_Motor_PosPP_control(
        kReturnZeroSpeed, kReturnZeroAcceleration, kReturnZeroTarget);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (hand_ && robstride_gripper_motor_) {
    robstride_gripper_motor_->RobStrite_Motor_PosPP_control(
        kReturnZeroSpeed, kReturnZeroAcceleration,
        static_cast<float>(joint_to_motor_radians(0.0)));
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
