#include "openarm_hardware/zero_torque_controller.hpp"

#include <algorithm>
#include <chrono>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace openarm_hardware
{

controller_interface::CallbackReturn ZeroTorqueController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
  auto_declare<double>("kd", 1.0);
  auto_declare<std::string>("urdf_path", "");
  auto_declare<std::string>("robot_description_node", "controller_manager");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ZeroTorqueController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "ZeroTorqueController: no joints configured");
    return controller_interface::CallbackReturn::ERROR;
  }

  kd_ = get_node()->get_parameter("kd").as_double();
  kd_ = std::clamp(kd_, 0.0, 5.0);

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

  RCLCPP_INFO(get_node()->get_logger(),
              "ZeroTorqueController configured: joints=%zu, kd=%.3f, pinocchio=%s",
              joint_names_.size(), kd_, pinocchio_ok_ ? "yes" : "no");

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
  RCLCPP_WARN(get_node()->get_logger(),
              "Zero-torque mode ACTIVATED (kd=%.3f, gravity=%s)",
              kd_, pinocchio_ok_ ? "on" : "off");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ZeroTorqueController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Zero-torque mode DEACTIVATED");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ZeroTorqueController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const size_t n = joint_names_.size();
  std::vector<double> gravity_torques(n, 0.0);

  if (pinocchio_ok_) {
    try {
      Eigen::VectorXd q = pinocchio::neutral(model_);
      Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
      Eigen::VectorXd a = Eigen::VectorXd::Zero(model_.nv);

      for (size_t i = 0; i < n; ++i) {
        const auto joint_id = joint_ids_[i];
        if (joint_id < model_.idx_qs.size()) {
          q[model_.idx_qs[joint_id]] = state_interfaces_[i * 2].get_value();
        }
      }

      Eigen::VectorXd tau = pinocchio::rnea(model_, data_, q, v, a);

      for (size_t i = 0; i < n; ++i) {
        const auto joint_id = joint_ids_[i];
        if (joint_id < model_.idx_vs.size()) {
          gravity_torques[i] = tau[model_.idx_vs[joint_id]];
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                           "ZeroTorqueController Pinocchio error: %s", e.what());
    }
  }

  for (size_t i = 0; i < n; ++i) {
    command_interfaces_[i].set_value(gravity_torques[i]);   //command_interfaces_ 是 controller_interface::ControllerInterface 提供的内置 protected 成员变量，用于将计算出的重力力矩写入硬件接口
  }

  static int pub_counter = 0;
  if (++pub_counter >= 10 && gravity_pub_) {
    pub_counter = 0;
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_node()->now();
    msg.name = joint_names_;
    msg.effort = gravity_torques;
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
                "ZeroTorqueController: robot_description not found on '%s'", target.c_str());
    return false;
  }

  const std::string urdf_xml = params[0].as_string();
  if (urdf_xml.empty()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "ZeroTorqueController: robot_description on '%s' is empty", target.c_str());
    return false;
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
