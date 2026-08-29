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

#include "openarm_hardware/zero_torque_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "std_msgs/msg/string.hpp"

namespace openarm_hardware
{

controller_interface::CallbackReturn ZeroTorqueController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
  auto_declare<double>("gravity_scale", 1.05);
  auto_declare<double>("coriolis_scale", 1.0);
  auto_declare<double>("kd", 0.0);  // 硬件层在示教模式切换的时候已经提供了PD控制的阻尼kd：zero_torque_kd=0.3，这里就不需要再叠加到控制器层了（tau_out += -kd_ * velocity），避免重复阻尼导致关节运动不灵敏
  auto_declare<std::vector<double>>("coulomb_friction", std::vector<double>{});
  auto_declare<std::vector<double>>("viscous_friction", std::vector<double>{});
  auto_declare<double>("friction_velocity_epsilon", 0.01);
  auto_declare<std::vector<double>>(
    "tau_limits", std::vector<double>{20.0, 20.0, 7.0, 7.0, 2.0, 2.0, 2.0});
  auto_declare<double>("hold_kp", 5.0);
  auto_declare<double>("hold_velocity_threshold", 0.05);
  auto_declare<double>("hold_timeout", 0.5);
  auto_declare<std::string>("urdf_path", "");
  auto_declare<std::string>("robot_description_node", "controller_manager");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ZeroTorqueController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array(); //获取当前控制器节点并从参数服务器(通常在启动控制器时，会加载一个.yaml配置文件)获取名为 "joints" 的参数
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "ZeroTorqueController: no joints configured");
    return controller_interface::CallbackReturn::ERROR;
  }

  const size_t n = joint_names_.size();

  gravity_scale_ = get_node()->get_parameter("gravity_scale").as_double();
  gravity_scale_ = std::clamp(gravity_scale_, 0.0, 2.0);

  coriolis_scale_ = get_node()->get_parameter("coriolis_scale").as_double();
  coriolis_scale_ = std::clamp(coriolis_scale_, 0.0, 2.0);

  kd_ = get_node()->get_parameter("kd").as_double();
  kd_ = std::clamp(kd_, 0.0, 10.0);

  // Friction parameters — pad to n with zeros if not fully specified
  coulomb_friction_ = get_node()->get_parameter("coulomb_friction").as_double_array();
  viscous_friction_ = get_node()->get_parameter("viscous_friction").as_double_array();
  friction_velocity_epsilon_ = get_node()->get_parameter("friction_velocity_epsilon").as_double();
  friction_velocity_epsilon_ = std::max(friction_velocity_epsilon_, 1e-6);
  while (coulomb_friction_.size() < n) coulomb_friction_.push_back(0.0);
  while (viscous_friction_.size() < n) viscous_friction_.push_back(0.0);

  // Torque limits — pad to n
  tau_limits_ = get_node()->get_parameter("tau_limits").as_double_array();
  while (tau_limits_.size() < n) tau_limits_.push_back(20.0);

  // Position hold parameters
  hold_kp_ = get_node()->get_parameter("hold_kp").as_double();
  hold_kp_ = std::clamp(hold_kp_, 0.0, 50.0);
  hold_velocity_threshold_ = get_node()->get_parameter("hold_velocity_threshold").as_double();
  hold_timeout_ = get_node()->get_parameter("hold_timeout").as_double();
  q_hold_.resize(n, 0.0);
  holding_ = false;

  urdf_path_ = get_node()->get_parameter("urdf_path").as_string();
  robot_description_node_ = get_node()->get_parameter("robot_description_node").as_string();

  bool loaded = false;
  if (!urdf_path_.empty()) {
    loaded = loadModelFromUrdfPath(urdf_path_);
  } else {
    loaded = loadModelFromRobotDescription(robot_description_node_);
  }

  pinocchio_ok_ = loaded && buildJointMap();

  gravity_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "~/gravity_torque", 10);

  // Runtime parameter callback (borrowed from openarmx architecture)
  param_callback_ = get_node()->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        if (p.get_name() == "gravity_scale") {
          gravity_scale_ = std::clamp(p.as_double(), 0.0, 2.0);
          RCLCPP_INFO(get_node()->get_logger(), "gravity_scale updated to %.3f", gravity_scale_);
        } else if (p.get_name() == "coriolis_scale") {
          coriolis_scale_ = std::clamp(p.as_double(), 0.0, 2.0);
          RCLCPP_INFO(get_node()->get_logger(), "coriolis_scale updated to %.3f", coriolis_scale_);
        } else if (p.get_name() == "kd") {
          kd_ = std::clamp(p.as_double(), 0.0, 10.0);
          RCLCPP_INFO(get_node()->get_logger(), "kd updated to %.3f", kd_);
        } else if (p.get_name() == "hold_kp") {
          hold_kp_ = std::clamp(p.as_double(), 0.0, 50.0);
          RCLCPP_INFO(get_node()->get_logger(), "hold_kp updated to %.3f", hold_kp_);
        }
      }
      return result;
    });

  RCLCPP_INFO(get_node()->get_logger(),
              "ZeroTorqueController configured: joints=%zu, kd=%.3f, gravity_scale=%.2f, "
              "coriolis_scale=%.2f, hold_kp=%.1f, pinocchio=%s",
              n, kd_, gravity_scale_, coriolis_scale_, hold_kp_,
              pinocchio_ok_ ? "yes" : "no");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ZeroTorqueController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    conf.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return conf;
}

controller_interface::InterfaceConfiguration
ZeroTorqueController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    conf.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return conf;
}

controller_interface::CallbackReturn ZeroTorqueController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset hold state on activation
  holding_ = false;
  RCLCPP_WARN(get_node()->get_logger(),
              "Zero-torque/teaching mode ACTIVATED "
              "(gravity_scale=%.2f, coriolis_scale=%.2f, kd=%.3f, hold_kp=%.1f, pinocchio=%s)",
              gravity_scale_, coriolis_scale_, kd_, hold_kp_,
              pinocchio_ok_ ? "on" : "off");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ZeroTorqueController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Zero-torque/teaching mode DEACTIVATED");
  return controller_interface::CallbackReturn::SUCCESS;
}


/*
τ_out = gravity_scale · g(q)                   // 重力补偿 (1.05)
      + coriolis_scale · C(q,v)·v              // Coriolis 补偿 (新增!)
      - tanh(v/ε) · τ_coulomb - v · b_viscous  // 摩擦补偿 (新增!)
      - kd_ · v                                // 控制器阻尼 (修复!)
      + hold_kp · (q_hold - q)                 // 松手定住 (新增!)
      clamp(±tau_limits)                        // 力矩限幅 (新增!)
*/
controller_interface::return_type ZeroTorqueController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  const size_t n = joint_names_.size();
  std::vector<double> output_torques(n, 0.0);

  if (pinocchio_ok_) {
    try {
      // 1. Read positions AND velocities (velocity is now actually used!)
      // 从状态接口读取位置（每两个接口一组：位置+速度）
      Eigen::VectorXd q = pinocchio::neutral(model_);
      Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
      for (size_t i = 0; i < n; ++i) {
        const auto jid = joint_ids_[i];
        if (jid < model_.idx_qs.size()) {
          q[model_.idx_qs[jid]] = state_interfaces_[i * 2].get_value();
        }
        if (jid < model_.idx_vs.size()) {
          v[model_.idx_vs[jid]] = state_interfaces_[i * 2 + 1].get_value();
        }
      }

      // 2. Compute dynamics: gravity and full bias force (gravity + Coriolis)
      Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(model_.nv);
      Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(model_.nv);
      Eigen::VectorXd tau_gravity = pinocchio::rnea(model_, data_, q, v_zero, a_zero);  // // 计算纯重力项：假设速度为0时的力矩（仅由重力产生的力矩）
      Eigen::VectorXd tau_bias = pinocchio::rnea(model_, data_, q, v, a_zero);          // // 计算完整偏置力：重力 + 科里奥利力 + 离心力（由速度引起的科里奥利力和离心力）
      // tau_coriolis = C(q,v)·v = tau_bias - tau_gravity
      Eigen::VectorXd tau_coriolis = tau_bias - tau_gravity;  //// 分离出科里奥利项（补偿这些力后，关节就"感觉"不到自身重量和惯性力了）

      // 3. Compose output torques for each joint
      for (size_t i = 0; i < n; ++i) {
        const auto jid = joint_ids_[i];
        const double vel = v[model_.idx_vs[jid]];
        double tau_out = 0.0;

        // Gravity compensation（补偿关节自身重力，微过补偿让手臂稍微上浮）
        tau_out += gravity_scale_ * tau_gravity[model_.idx_vs[jid]];

        // Coriolis compensation（补偿运动中的惯性耦合力，快速移动时感觉更轻）
        tau_out += coriolis_scale_ * tau_coriolis[model_.idx_vs[jid]];

        // Friction compensation: τ_f = -tanh(v/ε)·τ_coulomb - v·b_viscous（库伦摩擦+粘性摩擦，补偿后关节运动更丝滑）
        // tanh provides smooth transition near zero velocity
        if (i < coulomb_friction_.size() && coulomb_friction_[i] > 0.0) {
          tau_out += -std::tanh(vel / friction_velocity_epsilon_) * coulomb_friction_[i];  // 库伦摩擦力补偿（在低速时平滑过渡，避免抖动）
        }
        if (i < viscous_friction_.size() && viscous_friction_[i] > 0.0) {
          tau_out += -vel * viscous_friction_[i]; // 随速度增加而增大的阻力
        }

        // Controller-level damping: τ_d = -kd_ * v（控制器级阻尼，补偿关节运动的阻尼效果，防止关节乱飘）
        // (hardware zero_torque_kd provides motor-level damping separately)
        if (kd_ > 0.0) {
          tau_out += -kd_ * vel;
        }

        // Position hold: when velocity drops below threshold for hold_timeout seconds,
        // apply a weak position-holding spring to prevent drift
        if (hold_kp_ > 0.0) {
          // Compute max velocity across all joints (done once, cache for all joints)
          // We compute this on the first joint iteration and store
          if (i == 0) {
            double v_max = 0.0;
            for (size_t j = 0; j < n; ++j) {
              v_max = std::max(v_max, std::abs(v[model_.idx_vs[joint_ids_[j]]]));
            }

            if (v_max < hold_velocity_threshold_) {
              if (!holding_) {
                hold_start_time_ = time;
                for (size_t j = 0; j < n; ++j) {
                  q_hold_[j] = q[model_.idx_qs[joint_ids_[j]]];  // // 记录当前所有关节的位置作为目标位
                }
              }
              if ((time - hold_start_time_).seconds() > hold_timeout_) {
                holding_ = true;
              }
            } else {
              holding_ = false;
            }
          }

          if (holding_) {
            tau_out += hold_kp_ * (q_hold_[i] - q[model_.idx_qs[jid]]); // // 用弱弹簧把关节拉回记录的位置
          }
        }

        // Per-joint torque clamping (borrowed from openarmx)
        const double limit = (i < tau_limits_.size()) ? tau_limits_[i] : 20.0;
        tau_out = std::clamp(tau_out, -limit, limit);

        output_torques[i] = tau_out;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                           "ZeroTorqueController Pinocchio error: %s", e.what());
    }
  }

  // Write output torques to command interfaces
  for (size_t i = 0; i < n; ++i) {
    command_interfaces_[i].set_value(output_torques[i]);
  }

  // Debug publisher: publish computed torques to "~/gravity_torque"
  static int pub_counter = 0;
  if (++pub_counter >= 10 && gravity_pub_) {
    pub_counter = 0;
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_node()->now();
    msg.name = joint_names_;
    msg.effort = output_torques;
    gravity_pub_->publish(msg);
  }

  return controller_interface::return_type::OK;
}

bool ZeroTorqueController::loadModelFromUrdfPath(const std::string & urdf_path)
{
  try {
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    model_.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.81);
    RCLCPP_INFO(get_node()->get_logger(), "Loaded URDF: %s", urdf_path.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Pinocchio init failed for URDF '%s': %s", urdf_path.c_str(), e.what());
    return false;
  }
}

bool ZeroTorqueController::loadModelFromRobotDescription(const std::string & node_name)
{
  using namespace std::chrono_literals;

  std::string target = node_name;
  if (!target.empty() && target.front() != '/') {
    std::string ns = get_node()->get_namespace();
    if (ns == "/") {
      target = "/" + target;
    } else {
      target = ns + "/" + target;
    }
  }

  auto client_node = std::make_shared<rclcpp::Node>(
    get_node()->get_name() + std::string("_param_client"),
    get_node()->get_namespace());
  auto client = std::make_shared<rclcpp::SyncParametersClient>(client_node, target);
  if (!client->wait_for_service(2s)) {
    RCLCPP_WARN(get_node()->get_logger(),
                "ZeroTorqueController: parameter service not available on '%s'",
                target.c_str());
    return false;
  }

  auto params = client->get_parameters({"robot_description"});
  if (params.empty() || params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    RCLCPP_WARN(get_node()->get_logger(),
                "ZeroTorqueController: robot_description not found on '%s' parameters, "
                "falling back to latched /robot_description topic",
                target.c_str());
    return loadModelFromRobotDescriptionTopic("/robot_description");
  }

  const std::string urdf_xml = params[0].as_string();
  if (urdf_xml.empty()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "ZeroTorqueController: robot_description on '%s' is empty, "
                "falling back to latched /robot_description topic",
                target.c_str());
    return loadModelFromRobotDescriptionTopic("/robot_description");
  }

  try {
    pinocchio::urdf::buildModelFromXML(urdf_xml, model_);
    data_ = pinocchio::Data(model_);
    model_.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.81);
    RCLCPP_INFO(get_node()->get_logger(),
                "Loaded URDF from robot_description (%s)", target.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Pinocchio init failed for robot_description on '%s': %s",
                target.c_str(), e.what());
    return false;
  }
}

bool ZeroTorqueController::loadModelFromRobotDescriptionTopic(const std::string & topic)
{
  using namespace std::chrono_literals;

  // CM 在 topic 模式下收到 robot_description 后只初始化资源管理器，
  // 不会把它放到参数服务器上；此时直接订阅 robot_state_publisher
  // 锁存发布的 /robot_description（transient_local，先启动也能收到）
  auto client_node = std::make_shared<rclcpp::Node>(
    get_node()->get_name() + std::string("_desc_client"),
    get_node()->get_namespace());

  std::optional<std::string> urdf_xml;
  rclcpp::QoS latched_qos(1);
  latched_qos.transient_local();
  auto sub = client_node->create_subscription<std_msgs::msg::String>(
    topic, latched_qos,
    [&urdf_xml](const std_msgs::msg::String::SharedPtr msg) { urdf_xml = msg->data; });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client_node);
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (!urdf_xml && std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(100ms);
  }
  executor.remove_node(client_node);

  if (!urdf_xml || urdf_xml->empty()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "ZeroTorqueController: no robot_description received on '%s'", topic.c_str());
    return false;
  }

  try {
    pinocchio::urdf::buildModelFromXML(*urdf_xml, model_);
    data_ = pinocchio::Data(model_);
    model_.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.81);
    RCLCPP_INFO(get_node()->get_logger(),
                "Loaded URDF from latched topic '%s'", topic.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Pinocchio init failed for robot_description on '%s': %s",
                topic.c_str(), e.what());
    return false;
  }
}

bool ZeroTorqueController::buildJointMap()
{
  joint_ids_.clear();
  joint_ids_.reserve(joint_names_.size());

  for (const auto & name : joint_names_) {
    const auto joint_id = model_.getJointId(name);
    if (joint_id == 0) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "ZeroTorqueController: joint '%s' not found in URDF", name.c_str());
      return false;
    }
    joint_ids_.push_back(joint_id);
  }

  return true;
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(openarm_hardware::ZeroTorqueController,
                       controller_interface::ControllerInterface)
