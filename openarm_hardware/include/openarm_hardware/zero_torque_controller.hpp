#pragma once

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
  bool buildJointMap();

  std::vector<std::string> joint_names_;
  std::vector<pinocchio::JointIndex> joint_ids_;
  double kd_{1.0};

  std::string urdf_path_;
  std::string robot_description_node_;

  pinocchio::Model model_;
  pinocchio::Data data_;
  bool pinocchio_ok_{false};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gravity_pub_;
};

}  // namespace openarm_hardware
