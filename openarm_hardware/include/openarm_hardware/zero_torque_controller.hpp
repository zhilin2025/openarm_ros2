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

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace openarm_hardware
{

class ZeroTorqueController : public controller_interface::ControllerInterface
{
public:
  ZeroTorqueController() = default;
  ~ZeroTorqueController() override = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool loadModelFromUrdfPath(const std::string & urdf_path);
  bool loadModelFromRobotDescription(const std::string & node_name);
  bool loadModelFromRobotDescriptionTopic(const std::string & topic);
  bool buildJointMap();

  // Joint configuration
  std::vector<std::string> joint_names_;
  std::vector<pinocchio::JointIndex> joint_ids_;

  // Gravity compensation
  double gravity_scale_{1.05};  // Slightly over 1.0, borrowed from openarmx

  // Coriolis compensation
  double coriolis_scale_{1.0};

  // Controller-level damping (now actually used, separate from hardware zero_torque_kd)
  double kd_{0.0};

  // Friction compensation (Coulomb + viscous, tanh-smoothed)
  std::vector<double> coulomb_friction_;      // per joint [Nm]
  std::vector<double> viscous_friction_;      // per joint [Nm·s/rad]
  double friction_velocity_epsilon_{0.01};    // tanh smoothing width [rad/s]

  // Per-joint torque limits (borrowed from openarmx)
  std::vector<double> tau_limits_{20.0, 20.0, 7.0, 7.0, 2.0, 2.0, 2.0};

  // Position hold logic (lock position when user releases the arm)
  double hold_kp_{5.0};
  double hold_velocity_threshold_{0.05};  // [rad/s]
  double hold_timeout_{0.5};              // [s]
  std::vector<double> q_hold_;            // locked position
  rclcpp::Time hold_start_time_;          // when velocity first dropped below threshold
  bool holding_{false};

  // URDF / model
  std::string urdf_path_;
  std::string robot_description_node_;

  pinocchio::Model model_;
  pinocchio::Data data_;
  bool pinocchio_ok_{false};

  // Debug publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gravity_pub_;

  // Runtime parameter callback (borrowed from openarmx architecture)
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

}  // namespace openarm_hardware
