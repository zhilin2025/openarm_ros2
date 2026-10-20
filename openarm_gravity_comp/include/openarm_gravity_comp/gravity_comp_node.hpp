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

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace openarm_gravity_comp
{

class GravityCompNode : public rclcpp::Node
{
public:
  GravityCompNode();

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  bool loadModel(const std::string & urdf_path);

  // Parameters
  std::string urdf_path_;
  std::string output_topic_;
  double g_scale_{0.3};
  bool enable_compensation_{false};
  bool verbose_{false};
  std::vector<std::string> joint_names_;
  std::vector<double> tau_limits_{12.0, 12.0, 5.0, 5.0, 1.5, 1.5, 1.5};
  std::vector<double> torque_slew_rates_{3.0, 3.0, 1.5, 1.5, 0.5, 0.5, 0.5};
  std::vector<double> previous_output_torques_;
  rclcpp::Time previous_update_time_;
  rclcpp::Time previous_diagnostic_time_;

  // Pinocchio model
  pinocchio::Model model_;
  pinocchio::Data data_;
  std::vector<pinocchio::JointIndex> joint_ids_;
  bool pinocchio_ok_{false};

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

}  // namespace openarm_gravity_comp
