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

#include "openarm_gravity_comp/gravity_comp_node.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace openarm_gravity_comp
{

GravityCompNode::GravityCompNode() : Node("gravity_comp_node")
{
  // Declare parameters
  this->declare_parameter<std::string>("urdf_path", "/home/zl/openarm_ws/src/openarm_description/urdf/robot/openarm_v11.urdf");
  this->declare_parameter<std::string>(
    "output_topic", "/forward_effort_controller/commands");
  this->declare_parameter<double>("g_scale", 0.3);
  this->declare_parameter<bool>("enable_compensation", false);
  this->declare_parameter<bool>("verbose", false);
  this->declare_parameter<std::vector<std::string>>(
    "joint_names",
    std::vector<std::string>{
      "openarm_joint1", "openarm_joint2", "openarm_joint3",
      "openarm_joint4", "openarm_joint5", "openarm_joint6", "openarm_joint7"});
  this->declare_parameter<std::vector<double>>(
    "tau_limits", std::vector<double>{12.0, 12.0, 5.0, 5.0, 1.5, 1.5, 1.5});
  this->declare_parameter<std::vector<double>>(
    "torque_slew_rates", std::vector<double>{3.0, 3.0, 1.5, 1.5, 0.5, 0.5, 0.5});

  // Read parameters
  urdf_path_ = this->get_parameter("urdf_path").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  g_scale_ = this->get_parameter("g_scale").as_double();
  enable_compensation_ = this->get_parameter("enable_compensation").as_bool();
  verbose_ = this->get_parameter("verbose").as_bool();
  joint_names_ = this->get_parameter("joint_names").as_string_array();
  tau_limits_ = this->get_parameter("tau_limits").as_double_array();
  torque_slew_rates_ = this->get_parameter("torque_slew_rates").as_double_array();
  previous_output_torques_.assign(joint_names_.size(), 0.0);
  previous_update_time_ = this->now();
  previous_diagnostic_time_ = previous_update_time_;

  if (urdf_path_.empty()) {
    RCLCPP_FATAL(get_logger(), "Parameter 'urdf_path' is required but not set.");
    throw std::runtime_error("urdf_path not set");
  }
  if (output_topic_.empty()) {
    RCLCPP_FATAL(get_logger(), "Parameter 'output_topic' is required but not set.");
    throw std::runtime_error("output_topic not set");
  }

  // Load Pinocchio model
  pinocchio_ok_ = loadModel(urdf_path_);

  if (pinocchio_ok_) {
    RCLCPP_INFO(get_logger(),
                "GravityCompNode: loaded URDF '%s', %zu joints, g_scale=%.3f",
                urdf_path_.c_str(), joint_names_.size(), g_scale_);
  } else {
    RCLCPP_WARN(get_logger(),
                "GravityCompNode: Pinocchio model load failed, gravity compensation disabled");
  }

  // Publisher to forward_effort_controller
  torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    output_topic_, 10);

  // Subscriber to joint states
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&GravityCompNode::joint_state_callback, this, std::placeholders::_1));

  // Runtime parameter callback (borrowed from openarmx architecture)
  param_callback_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        if (p.get_name() == "g_scale") {
          g_scale_ = p.as_double();
          RCLCPP_INFO(get_logger(), "g_scale updated to %.3f", g_scale_);
        } else if (p.get_name() == "enable_compensation") {
          enable_compensation_ = p.as_bool();
          RCLCPP_INFO(get_logger(), "enable_compensation set to %s",
                      enable_compensation_ ? "true" : "false");
        }
      }
      return result;
    });

  RCLCPP_INFO(get_logger(),
              "GravityCompNode started. output='%s' g_scale=%.3f "
              "enable_compensation=%s pinocchio=%s",
              torque_pub_->get_topic_name(),
              g_scale_, enable_compensation_ ? "true" : "false",
              pinocchio_ok_ ? "ok" : "FAILED");
}

bool GravityCompNode::loadModel(const std::string & urdf_path)
{
  try {
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    model_.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.81);

    // Build joint ID mapping
    joint_ids_.clear();
    joint_ids_.reserve(joint_names_.size());
    for (const auto & name : joint_names_) {
      const auto jid = model_.getJointId(name);
      if (jid == 0) {
        RCLCPP_ERROR(get_logger(), "Joint '%s' not found in URDF", name.c_str());
        return false;
      }
      joint_ids_.push_back(jid);
    }

    // Pad tau_limits to match joint count
    while (tau_limits_.size() < joint_names_.size()) {
      tau_limits_.push_back(20.0);
    }

    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Pinocchio init failed for URDF '%s': %s",
                 urdf_path.c_str(), e.what());
    return false;
  }
}

void GravityCompNode::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  const size_t ndof = joint_names_.size();

  // When compensation is disabled, publish zeros to clear residual feedforward
  // (borrowed from openarmx architecture)
  if (!enable_compensation_ || !pinocchio_ok_) {
    auto out = std_msgs::msg::Float64MultiArray();
    out.data.assign(ndof, 0.0);
    torque_pub_->publish(out);
    std::fill(previous_output_torques_.begin(), previous_output_torques_.end(), 0.0);
    previous_update_time_ = this->now();
    return;
  }

  // Map joint_states (possibly unordered) into q[] by name lookup
  std::vector<double> q(ndof, 0.0);
  for (size_t j = 0; j < ndof; ++j) {
    auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[j]);
    if (it == msg->name.end()) {
      // Not all joints present yet — skip this cycle
      return;
    }
    size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
    if (idx >= msg->position.size()) {
      return;
    }
    q[j] = msg->position[idx];
  }

  // Build Pinocchio configuration vector
  Eigen::VectorXd q_pin = pinocchio::neutral(model_);
  for (size_t i = 0; i < ndof; ++i) {
    const auto jid = joint_ids_[i];
    if (jid < model_.idx_qs.size()) {
      q_pin[model_.idx_qs[jid]] = q[i];
    }
  }

  // Compute gravity torques: rnea(q, 0, 0) = g(q)
  Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(model_.nv);
  Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(model_.nv);

  std::vector<double> gravity_torques(ndof, 0.0);
  try {
    Eigen::VectorXd tau = pinocchio::rnea(model_, data_, q_pin, v_zero, a_zero);
    for (size_t i = 0; i < ndof; ++i) {
      const auto jid = joint_ids_[i];
      if (jid < model_.idx_vs.size()) {
        gravity_torques[i] = tau[model_.idx_vs[jid]];
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Pinocchio RNEA error: %s", e.what());
    auto out = std_msgs::msg::Float64MultiArray();
    out.data.assign(ndof, 0.0);
    torque_pub_->publish(out);
    return;
  }

  // Scale, clamp and rate-limit.  Compensation used to appear as a full
  // torque step as soon as it was enabled, which is unsafe when the model or
  // mounting direction has not yet been calibrated on the real arm.
  auto out = std_msgs::msg::Float64MultiArray();
  out.data.resize(ndof);
  const auto now = this->now();
  const double dt = std::clamp((now - previous_update_time_).seconds(), 0.0, 0.1);
  for (size_t j = 0; j < ndof; ++j) {
    double tau_motor = g_scale_ * gravity_torques[j];
    double limit = (j < tau_limits_.size()) ? tau_limits_[j]
                                             : std::numeric_limits<double>::infinity();
    tau_motor = std::clamp(tau_motor, -limit, limit);
    const double slew = j < torque_slew_rates_.size()
      ? std::max(0.05, torque_slew_rates_[j]) : 0.5;
    const double max_delta = slew * dt;
    tau_motor = std::clamp(
      tau_motor,
      previous_output_torques_[j] - max_delta,
      previous_output_torques_[j] + max_delta);
    out.data[j] = tau_motor;
    previous_output_torques_[j] = tau_motor;

    if (verbose_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "j%zu q=%.3f tau_g=%.3f tau_out=%.3f",
                           j, q[j], gravity_torques[j], tau_motor);
    }
  }
  previous_update_time_ = now;

  torque_pub_->publish(out);

  // Do not silently run an apparently healthy compensation node when its
  // command topic is disconnected.  This exact condition previously made
  // changing g_scale from 0.30 to 0.70 have no effect on the real arm.
  if ((now - previous_diagnostic_time_).seconds() >= 2.0) {
    previous_diagnostic_time_ = now;
    const size_t subscribers = torque_pub_->get_subscription_count();
    const double max_abs_torque = out.data.empty() ? 0.0 :
      std::abs(*std::max_element(
        out.data.begin(), out.data.end(),
        [](double lhs, double rhs) {return std::abs(lhs) < std::abs(rhs);}));
    if (subscribers == 0) {
      RCLCPP_ERROR(
        get_logger(),
        "Gravity feedforward DISCONNECTED: output='%s' has no subscribers",
        torque_pub_->get_topic_name());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Gravity feedforward connected: output='%s' subscribers=%zu "
        "max_abs_tau=%.3f Nm",
        torque_pub_->get_topic_name(), subscribers, max_abs_torque);
    }
  }
}

}  // namespace openarm_gravity_comp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<openarm_gravity_comp::GravityCompNode>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("gravity_comp_node"), "Fatal: %s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
