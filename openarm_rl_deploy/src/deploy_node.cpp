#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <torch/script.h>  // LibTorch
#include <torch/torch.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "openarmx_deploy/arm_kinematics.h"
#include "openarmx_deploy/grasp_state_machine.h"
#include "openarmx_deploy/observation_builder.h"

namespace openarmx_deploy {

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleFollowJointTrajectory =
    rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

// ============================================================================
// OpenArmXDeployNode — RL 抓取策略部署节点
//
// 订阅 /yolo_detection/object_poses (PoseArray) 获取物体位置，订阅
// /joint_states 获取关节角，通过 ros2_control 的 Action 接口下发轨迹/夹爪
// 命令（与 GUI openarm_visual_controller.py 使用的控制器一致）。
// ============================================================================
class OpenArmXDeployNode : public rclcpp::Node {
 public:
  OpenArmXDeployNode();
  ~OpenArmXDeployNode() override;

 private:
  // ---- ROS 回调 ----
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onObjectPoses(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  // ---- 服务 ----
  void handleGrasp(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleReset(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // ---- 核心逻辑 ----
  void runGraspLoop();
  std::vector<float> buildObservation();
  std::vector<float> policyAction(const std::vector<float>& obs);
  void executeAction(const std::vector<float>& action);

  // ---- 辅助 ----
  std::array<double, 3> getCurrentTCP();   // 由 FK 计算真实 TCP 位置
  bool solveIK(const std::array<double, 3>& target_pos,
               const std::vector<double>& initial_q,
               std::vector<double>& out_q);
  void sendArmGoal(const std::vector<double>& start_q,
                   const std::vector<double>& target_q);
  void sendGripperGoal(double position);
  std::vector<double> currentArmQ();       // 线程安全地读取当前臂关节角

  // ---- 回初始位姿 ----
  void moveToHome();
  bool waitForHome(double timeout_s);
  void interpolateToHome();

  // ---- 参数 ----
  void loadParameters();

  // ---- ROS 接口 ----
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr object_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr traj_client_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr grasp_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---- 模型 ----
  torch::jit::script::Module actor_;

  // ---- 运行时状态 ----
  std::mutex mutex_;
  std::map<std::string, double> arm_qpos_;
  std::map<std::string, double> gripper_qpos_;
  double object_x_ = 0.0, object_y_ = 0.0, object_z_ = 0.0;
  std::atomic_bool joints_received_{false};
  std::atomic_bool object_received_{false};
  rclcpp::Time object_stamp_{0, 0, RCL_ROS_TIME};

  // ---- 状态机 & 控制 ----
  ObservationBuilder obs_builder_;
  std::unique_ptr<GraspStateMachine> state_machine_;
  int step_count_ = 0;
  std::atomic_bool grasp_active_{false};
  std::atomic_bool arm_goal_active_{false};
  std::atomic_uint64_t arm_goal_id_{0};
  bool grasp_success_ = false;
  std::thread grasp_thread_;
  bool grasp_thread_running_ = false;

  // ---- 初始位姿 (Home) ----
  std::vector<double> home_qpos_;
  bool home_recorded_ = false;
  std::vector<double> home_pose_param_;
  double home_tolerance_ = 0.02;
  double home_timeout_ = 30.0;
  int home_interp_steps_ = 50;

  // ---- 运动学 ----
  std::unique_ptr<ArmKinematics> kin_;

  // ---- 参数 ----
  std::string arm_side_ = "right";
  std::vector<std::string> arm_joint_names_;
  std::string gripper_joint_name_;
  double gripper_open_ = -0.042;   // v11 真实夹爪 openarm_*_finger_joint1 打开位置
  double gripper_close_ = 0.0;     // 下限
  std::string object_pose_topic_ = "/yolo_detection/object_poses";
  std::string base_frame_ = "openarm_body_link0";
  double object_timeout_ = 1.0;
  std::string model_path_;
  int max_steps_ = 400;
  double action_pos_scale_ = 0.03;
  double action_gripper_scale_ = 0.005;
  double control_rate_ = 30.0;
  double max_joint_velocity_ = 0.50;
  double min_arm_goal_duration_ = 0.25;
  double gripper_goal_epsilon_ = 0.0005;
  double last_gripper_goal_ = std::numeric_limits<double>::quiet_NaN();
  double approach_height_ = 0.15;
  double grasp_height_offset_ = 0.008;
  double lift_height_ = 0.12;
  double grasp_yaw_ = 0.0;
  double table_z_ = 0.80;
  int close_steps_ = 30;
  int settle_steps_ = 20;
  int lift_steps_ = 40;
};

// --------------------------------------------------------------------------
// 构造
// --------------------------------------------------------------------------
OpenArmXDeployNode::OpenArmXDeployNode()
    : Node("openarmx_deploy") {
  loadParameters();

  kin_ = std::make_unique<ArmKinematics>(
      arm_side_ == "left" ? ArmKinematics::Side::kLeft : ArmKinematics::Side::kRight);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ---- 加载 TorchScript 模型 ----
  if (model_path_.empty()) {
    throw std::runtime_error("Parameter 'checkpoint' must not be empty");
  } else {
    try {
      actor_ = torch::jit::load(model_path_);
      actor_.eval();
      RCLCPP_INFO(get_logger(), "Loaded TorchScript model: %s", model_path_.c_str());
    } catch (const c10::Error& e) {
      RCLCPP_FATAL(get_logger(), "Failed to load model: %s", e.what());
      throw;
    }
  }

  // 验证模型: 跑一次 dummy 推理
  {
    auto dummy = torch::randn({1, ObservationBuilder::kObservationDim});
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(dummy);
    auto output = actor_.forward(inputs).toTensor();
    if (output.dim() != 2 || output.size(0) != 1 || output.size(1) != 4) {
      throw std::runtime_error("Policy output must have shape [1, 4]");
    }
    RCLCPP_INFO(get_logger(), "Model test inference: output shape [%ld, %ld]",
                output.size(0), output.size(1));
  }

  RCLCPP_INFO(get_logger(), "Arm side: %s, %zu arm joints, gripper '%s'",
              arm_side_.c_str(), arm_joint_names_.size(), gripper_joint_name_.c_str());

  // ---- 订阅 ----
  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&OpenArmXDeployNode::onJointState, this, std::placeholders::_1));
  object_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      object_pose_topic_, 10,
      std::bind(&OpenArmXDeployNode::onObjectPoses, this, std::placeholders::_1));

  // ---- Action 客户端 (镜像 GUI 的控制接口) ----
  traj_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/" + arm_side_ + "_joint_trajectory_controller/follow_joint_trajectory");
  gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      this, "/" + arm_side_ + "_gripper_controller/gripper_cmd");

  // ---- 服务 ----
  grasp_srv_ = create_service<std_srvs::srv::Trigger>(
      "/openarmx_grasp",
      std::bind(&OpenArmXDeployNode::handleGrasp, this,
                std::placeholders::_1, std::placeholders::_2));
  reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "/openarmx_reset",
      std::bind(&OpenArmXDeployNode::handleReset, this,
                std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "OpenArmX deploy node started. control_rate=%.1f Hz",
              control_rate_);
}

OpenArmXDeployNode::~OpenArmXDeployNode() {
  grasp_active_ = false;
  if (grasp_thread_.joinable()) grasp_thread_.join();
}

// --------------------------------------------------------------------------
// 参数加载
// --------------------------------------------------------------------------
void OpenArmXDeployNode::loadParameters() {
  declare_parameter("arm_side", "right");
  declare_parameter("object_pose_topic", "/yolo_detection/object_poses");
  declare_parameter("base_frame", "openarm_body_link0");
  declare_parameter("object_timeout", 1.0);
  declare_parameter("checkpoint", "models/jgzh_sim2real.pt");  // 相对路径按包 share 目录解析
  declare_parameter("max_steps", 400);
  declare_parameter("action_pos_scale", 0.03);
  declare_parameter("action_gripper_scale", 0.005);
  declare_parameter("control_rate", 30.0);
  declare_parameter("max_joint_velocity", 0.50);
  declare_parameter("min_arm_goal_duration", 0.25);
  declare_parameter("gripper_goal_epsilon", 0.0005);
  declare_parameter("approach_height", 0.15);
  declare_parameter("grasp_height_offset", 0.008);
  declare_parameter("lift_height", 0.12);
  declare_parameter("grasp_yaw", 0.0);
  declare_parameter("table_z", 0.80);
  declare_parameter("close_steps", 30);
  declare_parameter("settle_steps", 20);
  declare_parameter("lift_steps", 40);
  declare_parameter("home_pose", "");     // 固定初始关节角 (逗号分隔 7 个), 空=启动时记录
  declare_parameter("home_tolerance", 0.02);
  declare_parameter("home_timeout", 30.0);
  declare_parameter("home_interp_steps", 50);

  arm_side_ = get_parameter("arm_side").as_string();
  if (arm_side_ != "left" && arm_side_ != "right") {
    RCLCPP_WARN(get_logger(), "Unknown arm_side '%s', using 'right'.", arm_side_.c_str());
    arm_side_ = "right";
  }

  // 关节名: openarm_<side>_joint1..7 + openarm_<side>_finger_joint1
  arm_joint_names_.clear();
  for (int i = 1; i <= 7; ++i) {
    arm_joint_names_.push_back("openarm_" + arm_side_ + "_joint" + std::to_string(i));
  }
  gripper_joint_name_ = "openarm_" + arm_side_ + "_finger_joint1";

  object_pose_topic_ = get_parameter("object_pose_topic").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  object_timeout_ = get_parameter("object_timeout").as_double();

  model_path_ = get_parameter("checkpoint").as_string();
  if (!model_path_.empty() && model_path_[0] != '/') {
    // 相对路径 -> 包 share 目录 (models/ 已随包安装)
    try {
      std::string share = ament_index_cpp::get_package_share_directory("openarmx_deploy");
      model_path_ = share + "/" + model_path_;
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Cannot resolve share dir: %s", e.what());
    }
  }

  max_steps_ = get_parameter("max_steps").as_int();
  action_pos_scale_ = get_parameter("action_pos_scale").as_double();
  action_gripper_scale_ = get_parameter("action_gripper_scale").as_double();
  control_rate_ = get_parameter("control_rate").as_double();
  max_joint_velocity_ = get_parameter("max_joint_velocity").as_double();
  min_arm_goal_duration_ = get_parameter("min_arm_goal_duration").as_double();
  gripper_goal_epsilon_ = get_parameter("gripper_goal_epsilon").as_double();
  if (max_joint_velocity_ <= 0.0 || min_arm_goal_duration_ <= 0.0 ||
      gripper_goal_epsilon_ < 0.0) {
    throw std::runtime_error(
        "max_joint_velocity and min_arm_goal_duration must be positive; "
        "gripper_goal_epsilon must not be negative");
  }
  approach_height_ = get_parameter("approach_height").as_double();
  grasp_height_offset_ = get_parameter("grasp_height_offset").as_double();
  lift_height_ = get_parameter("lift_height").as_double();
  grasp_yaw_ = get_parameter("grasp_yaw").as_double();
  table_z_ = get_parameter("table_z").as_double();
  close_steps_ = get_parameter("close_steps").as_int();
  settle_steps_ = get_parameter("settle_steps").as_int();
  lift_steps_ = get_parameter("lift_steps").as_int();

  {
    auto home_pose_str = get_parameter("home_pose").as_string();
    if (!home_pose_str.empty()) {
      std::stringstream ss(home_pose_str);
      std::string token;
      while (std::getline(ss, token, ',')) {
        home_pose_param_.push_back(std::stod(token));
      }
    }
  }
  home_tolerance_ = get_parameter("home_tolerance").as_double();
  home_timeout_ = get_parameter("home_timeout").as_double();
  home_interp_steps_ = get_parameter("home_interp_steps").as_int();

  if (!home_pose_param_.empty()) {
    if (home_pose_param_.size() != arm_joint_names_.size()) {
      throw std::runtime_error("Parameter 'home_pose' must contain exactly 7 values");
    }
    home_qpos_ = home_pose_param_;
    home_recorded_ = true;
    RCLCPP_INFO(get_logger(), "Home pose set from parameter (%zu joints).",
                home_qpos_.size());
  } else {
    RCLCPP_INFO(get_logger(), "No home_pose param — will record initial joint states as home.");
  }
}

// --------------------------------------------------------------------------
// ROS 回调
// --------------------------------------------------------------------------
void OpenArmXDeployNode::onJointState(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    for (const auto& name : arm_joint_names_) {
      if (msg->name[i] == name) arm_qpos_[name] = msg->position[i];
    }
    if (msg->name[i] == gripper_joint_name_) {
      gripper_qpos_[gripper_joint_name_] = msg->position[i];
    }
  }
  joints_received_ = std::all_of(
      arm_joint_names_.begin(), arm_joint_names_.end(),
      [this](const std::string& name) { return arm_qpos_.count(name) != 0; });

  // 记录初始位姿: 首次收到全部臂关节状态时保存为 home (若参数未指定固定位姿)
  if (!home_recorded_ && home_pose_param_.empty()) {
    bool all_present = true;
    for (const auto& name : arm_joint_names_) {
      if (!arm_qpos_.count(name)) {
        all_present = false;
        break;
      }
    }
    if (all_present) {
      home_qpos_.clear();
      for (const auto& name : arm_joint_names_) {
        home_qpos_.push_back(arm_qpos_[name]);
      }
      home_recorded_ = true;
      RCLCPP_INFO(get_logger(), "Home pose recorded from initial joint states.");
    }
  }
}

void OpenArmXDeployNode::onObjectPoses(
    const geometry_msgs::msg::PoseArray::SharedPtr msg) {
  if (msg->poses.empty()) return;
  if (msg->header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Ignoring object pose with an empty frame_id.");
    return;
  }

  geometry_msgs::msg::PointStamped source;
  source.header = msg->header;
  source.point = msg->poses[0].position;
  geometry_msgs::msg::PointStamped target;
  try {
    target = tf_buffer_->transform(source, base_frame_, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Cannot transform object from '%s' to '%s': %s",
                         msg->header.frame_id.c_str(), base_frame_.c_str(), e.what());
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const auto& p = target.point;
  object_x_ = p.x;
  object_y_ = p.y;
  object_z_ = p.z;
  object_stamp_ = now();
  object_received_ = true;
}

// --------------------------------------------------------------------------
// 服务
// --------------------------------------------------------------------------
void OpenArmXDeployNode::handleGrasp(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (grasp_active_.load()) {
    response->success = false;
    response->message = "Grasp already in progress";
    return;
  }
  if (!joints_received_.load() || !object_received_.load()) {
    response->success = false;
    response->message = "Waiting for /joint_states and object poses";
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if ((now() - object_stamp_).seconds() > object_timeout_) {
      response->success = false;
      response->message = "Object pose is stale";
      return;
    }
  }
  if (!traj_client_->action_server_is_ready() ||
      !gripper_client_->action_server_is_ready()) {
    response->success = false;
    response->message = "Arm or gripper controller action server is not ready";
    return;
  }
  if (grasp_thread_running_ && grasp_thread_.joinable()) {
    grasp_thread_.join();
    grasp_thread_running_ = false;
  }
  grasp_active_ = true;
  grasp_success_ = false;
  response->success = true;
  response->message = "Grasp started";

  grasp_thread_ = std::thread([this]() { runGraspLoop(); });
  grasp_thread_running_ = true;
}

void OpenArmXDeployNode::handleReset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  grasp_active_ = false;
  if (grasp_thread_running_ && grasp_thread_.joinable()) {
    grasp_thread_.join();
    grasp_thread_running_ = false;
  }

  moveToHome();

  state_machine_.reset();
  step_count_ = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    object_received_ = false;
  }
  response->success = true;
  response->message = "Reset complete";
  RCLCPP_INFO(get_logger(), "Reset complete.");
}

// --------------------------------------------------------------------------
// 核心抓取循环
// --------------------------------------------------------------------------
// 30Hz 控制循环：每步重新观测、重新推理、微调目标并执行（RL 伺服闭环）。
// 类似于单点伺服（servo-to-pose）+ RL 微调。每步执行的目标位姿由 expert 规划 + RL 输出修正量组成。
void OpenArmXDeployNode::runGraspLoop() {
  RCLCPP_INFO(get_logger(), "Starting grasp loop...");
  step_count_ = 0;
  last_gripper_goal_ = std::numeric_limits<double>::quiet_NaN();

  // Invalidate callbacks left over from a previous grasp/reset session.
  arm_goal_id_.fetch_add(1);
  arm_goal_active_ = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::array<double, 3> obj_pos = {object_x_, object_y_, object_z_};
    state_machine_ = std::make_unique<GraspStateMachine>(
        obj_pos, approach_height_, grasp_height_offset_, lift_height_,
        /*table_clearance=*/0.02, table_z_, grasp_yaw_,
        close_steps_, settle_steps_, lift_steps_,
        gripper_open_, gripper_close_);
  }

  rclcpp::Rate rate(control_rate_);

  while (grasp_active_.load() && rclcpp::ok()) {
    if (step_count_ >= max_steps_) {
      RCLCPP_WARN(get_logger(), "Max steps (%d) reached. Stopping.", max_steps_);
      break;
    }

    if (state_machine_->isDone() && !arm_goal_active_.load()) {
      grasp_success_ = true;
      RCLCPP_INFO(get_logger(), "Grasp motion sequence completed.");
      break;
    }

    auto obs = buildObservation();
    auto action = policyAction(obs);
    executeAction(action);

    step_count_++;
    rate.sleep();
  }

  grasp_active_ = false;

  if (grasp_success_) {
    RCLCPP_INFO(get_logger(),
                "Grasp sequence completed (object pickup is not sensor-verified), steps: %d",
                step_count_);
  } else {
    RCLCPP_WARN(get_logger(), "Grasp STOPPED — total steps: %d", step_count_);
  }
}

// --------------------------------------------------------------------------
// 观测构造
// --------------------------------------------------------------------------
std::vector<float> OpenArmXDeployNode::buildObservation() {
  std::vector<double> arm_q = currentArmQ();
  double grip = 0.0;
  double ox, oy, oz;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    grip = gripper_qpos_.count(gripper_joint_name_)
               ? gripper_qpos_[gripper_joint_name_] : 0.0;
    ox = object_x_; oy = object_y_; oz = object_z_;
  }

  auto tcp = kin_->forwardPos(arm_q);

  obs_builder_.updateArmQpos(arm_q);
  // 真实夹爪为单指令关节 + mimic 指; 观测 2 维夹爪槽均填入同一开度
  obs_builder_.updateGripperQpos({grip, grip});
  obs_builder_.updateObjectPos(ox, oy, oz);
  obs_builder_.updateFingerpadCenter(tcp[0], tcp[1], tcp[2]);

  std::array<double, 3> obj_arr = {ox, oy, oz};
  std::array<double, 3> target_pos;
  std::array<double, 9> target_xmat;
  double gripper_target;
  state_machine_->getTarget(obj_arr, target_pos, target_xmat, gripper_target);

  obs_builder_.updateTarget(target_pos, gripper_target);
  obs_builder_.updateState(step_count_, max_steps_, state_machine_->oneHot());

  return obs_builder_.build();
}

// --------------------------------------------------------------------------
// 策略推理
// --------------------------------------------------------------------------
// 输出修正量: 相对于 expert 目标的增量 ([-1,1] 映射到实际增量), 粗规划+细调整。
std::vector<float> OpenArmXDeployNode::policyAction(const std::vector<float>& obs) {
  auto tensor = torch::from_blob(
      const_cast<float*>(obs.data()),
      {1, static_cast<long>(ObservationBuilder::kObservationDim)},
      torch::kFloat32).clone();

  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(tensor);

  torch::NoGradGuard no_grad;
  auto output = actor_.forward(inputs).toTensor();

  auto acc = output.accessor<float, 2>();
  std::vector<float> action(4);
  for (int i = 0; i < 4; ++i) {
    action[i] = std::clamp(acc[0][i], -1.0f, 1.0f);
  }
  return action;
}

// --------------------------------------------------------------------------
// 执行动作
// --------------------------------------------------------------------------
void OpenArmXDeployNode::executeAction(const std::vector<float>& action) {
  std::vector<double> arm_q = currentArmQ();
  double ox, oy, oz;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ox = object_x_; oy = object_y_; oz = object_z_;
  }

  // 目标位姿 (expert + 策略修正)
  std::array<double, 3> tar_pos, obj_arr = {ox, oy, oz};
  std::array<double, 9> tar_xmat;
  double expert_gripper;
  state_machine_->getTarget(obj_arr, tar_pos, tar_xmat, expert_gripper);

  tar_pos[0] += action[0] * action_pos_scale_;
  tar_pos[1] += action[1] * action_pos_scale_;
  tar_pos[2] += action[2] * action_pos_scale_;

  double grip_target = std::clamp(
      expert_gripper + action[3] * action_gripper_scale_,
      gripper_close_, gripper_open_);

  // 状态机推进
  auto actual = getCurrentTCP();
  double pos_err = std::sqrt(
      (tar_pos[0] - actual[0]) * (tar_pos[0] - actual[0]) +
      (tar_pos[1] - actual[1]) * (tar_pos[1] - actual[1]) +
      (tar_pos[2] - actual[2]) * (tar_pos[2] - actual[2]));
  double xy_err = std::sqrt(
      (tar_pos[0] - actual[0]) * (tar_pos[0] - actual[0]) +
      (tar_pos[1] - actual[1]) * (tar_pos[1] - actual[1]));
  double z_err = std::abs(tar_pos[2] - actual[2]);
  state_machine_->update(pos_err, xy_err, z_err);

  if (std::isnan(last_gripper_goal_) ||
      std::abs(grip_target - last_gripper_goal_) >= gripper_goal_epsilon_) {
    sendGripperGoal(grip_target);
    last_gripper_goal_ = grip_target;
  }

  // Keep one trajectory segment in flight. The policy still runs at
  // control_rate_, but a new IK target is sent only after JTC finishes the
  // previous segment, preventing a stream of 33 ms preempting goals.
  if (!arm_goal_active_.load()) {
    std::vector<double> ik_q;
    if (solveIK(tar_pos, arm_q, ik_q)) {
      sendArmGoal(arm_q, ik_q);
    }
  }
}

// --------------------------------------------------------------------------
// 辅助 — TCP / IK / 发布
// --------------------------------------------------------------------------
std::vector<double> OpenArmXDeployNode::currentArmQ() {
  std::vector<double> arm_q(7, 0.0);
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < arm_joint_names_.size(); ++i) {
    const auto& name = arm_joint_names_[i];
    arm_q[i] = arm_qpos_.count(name) ? arm_qpos_[name] : 0.0;
  }
  return arm_q;
}

std::array<double, 3> OpenArmXDeployNode::getCurrentTCP() {
  return kin_->forwardPos(currentArmQ());
}

bool OpenArmXDeployNode::solveIK(const std::array<double, 3>& target_pos,
                                 const std::vector<double>& initial_q,
                                 std::vector<double>& out_q) {
  bool converged = kin_->inversePosition(target_pos, initial_q, out_q);
  if (!converged) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "IK did not converge; holding the current arm command.");
  }
  return converged;
}

void OpenArmXDeployNode::sendArmGoal(const std::vector<double>& start_q,
                                     const std::vector<double>& target_q) {
  if (!traj_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Trajectory action server not ready (controller not started?).");
    return;
  }
  if (start_q.size() != arm_joint_names_.size() ||
      target_q.size() != arm_joint_names_.size()) {
    RCLCPP_ERROR(get_logger(),
                 "Cannot send arm goal: expected %zu joints, got start=%zu target=%zu.",
                 arm_joint_names_.size(), start_q.size(), target_q.size());
    return;
  }

  bool expected = false;
  if (!arm_goal_active_.compare_exchange_strong(expected, true)) {
    return;
  }

  double max_delta = 0.0;
  for (size_t i = 0; i < target_q.size(); ++i) {
    max_delta = std::max(max_delta, std::abs(target_q[i] - start_q[i]));
  }
  // With zero endpoint velocities, JTC's cubic interpolation reaches a peak
  // speed of 1.5 * delta / duration for the largest-moving joint.
  const double duration_s = std::max(
      min_arm_goal_duration_, 1.5 * max_delta / max_joint_velocity_);

  auto goal = FollowJointTrajectory::Goal();
  goal.trajectory.joint_names = arm_joint_names_;
  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  start_point.positions = start_q;
  start_point.velocities.assign(start_q.size(), 0.0);
  start_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
  goal.trajectory.points.push_back(start_point);

  trajectory_msgs::msg::JointTrajectoryPoint target_point;
  target_point.positions = target_q;
  target_point.velocities.assign(target_q.size(), 0.0);
  target_point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
  goal.trajectory.points.push_back(target_point);

  const uint64_t goal_id = arm_goal_id_.fetch_add(1) + 1;
  auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  options.goal_response_callback =
      [this, goal_id](GoalHandleFollowJointTrajectory::SharedPtr goal_handle) {
        if (!goal_handle && arm_goal_id_.load() == goal_id) {
          arm_goal_active_ = false;
          RCLCPP_WARN(get_logger(), "Arm trajectory goal was rejected.");
        }
      };
  options.result_callback =
      [this, goal_id](const GoalHandleFollowJointTrajectory::WrappedResult& result) {
        if (arm_goal_id_.load() != goal_id) {
          return;
        }
        arm_goal_active_ = false;
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(get_logger(), "Arm trajectory ended with result code %d.",
                      static_cast<int>(result.code));
        }
      };

  try {
    traj_client_->async_send_goal(goal, options);
    RCLCPP_INFO(get_logger(),
                "Arm trajectory sent: max_delta=%.3f rad, duration=%.3f s.",
                max_delta, duration_s);
  } catch (const std::exception& e) {
    if (arm_goal_id_.load() == goal_id) {
      arm_goal_active_ = false;
    }
    RCLCPP_ERROR(get_logger(), "Failed to send arm trajectory: %s", e.what());
  }
}

void OpenArmXDeployNode::sendGripperGoal(double position) {
  if (!gripper_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Gripper action server not ready (controller not started?).");
    return;
  }

  auto goal = GripperCommand::Goal();
  goal.command.position = position;
  goal.command.max_effort = 1.0;

  auto options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
  gripper_client_->async_send_goal(goal, options);
}

// --------------------------------------------------------------------------
// 回初始位姿
// --------------------------------------------------------------------------
void OpenArmXDeployNode::moveToHome() {
  if (!home_recorded_) {
    RCLCPP_WARN(get_logger(),
                "Home pose not available (no joint states yet / no home_pose param). Skipping.");
    return;
  }
  if (home_qpos_.size() != arm_joint_names_.size()) {
    RCLCPP_WARN(get_logger(), "Home pose size (%zu) != arm joint count (%zu). Skipping.",
                home_qpos_.size(), arm_joint_names_.size());
    return;
  }

  std::vector<double> current = currentArmQ();

  double max_err = 0.0;
  for (size_t i = 0; i < current.size(); ++i) {
    max_err = std::max(max_err, std::abs(current[i] - home_qpos_[i]));
  }
  if (max_err < home_tolerance_) {
    RCLCPP_INFO(get_logger(), "Already at home pose (max_err=%.4f rad).", max_err);
    return;
  }

  interpolateToHome();

  if (waitForHome(home_timeout_)) {
    RCLCPP_INFO(get_logger(), "Reached home pose.");
  } else {
    RCLCPP_WARN(get_logger(), "Home move timed out after %.1f s.", home_timeout_);
  }
}

// --------------------------------------------------------------------------
// 关节空间线性插值回位 (多点轨迹, 经 Action 发送)
// --------------------------------------------------------------------------
void OpenArmXDeployNode::interpolateToHome() {
  if (!traj_client_->action_server_is_ready()) {
    RCLCPP_WARN(get_logger(), "Trajectory action server not ready — cannot move home.");
    return;
  }

  std::vector<double> current = currentArmQ();

  auto goal = FollowJointTrajectory::Goal();
  goal.trajectory.joint_names = arm_joint_names_;
  double step_duration = 1.0 / control_rate_;
  for (int i = 1; i <= home_interp_steps_; ++i) {
    double t = static_cast<double>(i) / home_interp_steps_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.reserve(current.size());
    for (size_t j = 0; j < current.size(); ++j) {
      point.positions.push_back(current[j] + t * (home_qpos_[j] - current[j]));
    }
    point.time_from_start = rclcpp::Duration::from_seconds(i * step_duration);
    goal.trajectory.points.push_back(point);
  }
  traj_client_->async_send_goal(goal);
  RCLCPP_INFO(get_logger(), "Interpolating to home pose (%d steps)...", home_interp_steps_);
}

bool OpenArmXDeployNode::waitForHome(double timeout_s) {
  if (home_qpos_.size() != arm_joint_names_.size()) return false;
  rclcpp::Time start = now();
  while (rclcpp::ok()) {
    double max_err = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (size_t i = 0; i < arm_joint_names_.size(); ++i) {
        const auto& name = arm_joint_names_[i];
        if (!arm_qpos_.count(name)) return false;
        max_err = std::max(max_err, std::abs(arm_qpos_[name] - home_qpos_[i]));
      }
    }
    if (max_err < home_tolerance_) return true;
    if ((now() - start).seconds() > timeout_s) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return false;
}

}  // namespace openarmx_deploy

// ============================================================================
// main
// ============================================================================
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<openarmx_deploy::OpenArmXDeployNode>();
    // 多线程 executor: 服务回调 (reset 会 join 抓取线程并轮询关节状态) 与
    // 话题/action 回调并发处理，避免单线程 spin 死锁。
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4u);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("openarmx_deploy"), "Fatal: %s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
