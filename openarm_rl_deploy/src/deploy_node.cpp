#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <torch/script.h>  // LibTorch
#include <torch/torch.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "openarmx_deploy/grasp_state_machine.h"
#include "openarmx_deploy/observation_builder.h"

namespace openarmx_deploy {

// ============================================================================
// 机器人配置 — 各型号的关节名、夹爪参数
// ============================================================================
struct RobotConfig {
  std::vector<std::string> arm_joint_names;
  std::vector<std::string> gripper_joint_names;
  double gripper_open;
  double gripper_close;
};

static const RobotConfig kRobotConfigs[] = {
  // JGZH — 7-DOF 臂 + 2 指 prismatic gripper
  {
    {"JGZH_joint1", "JGZH_joint2", "JGZH_joint3", "JGZH_joint4",
     "JGZH_joint5", "JGZH_joint6", "JGZH_joint7"},
    {"JGZH_left_finger_joint", "JGZH_right_finger_joint"},
    0.01, -0.018
  },
  // OpenArmX — 7-DOF 臂 + 2 指 prismatic gripper
  {
    {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
     "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "wrist_4_joint"},
    {"left_finger_joint", "right_finger_joint"},
    0.044, 0.0
  },
  // Sciurus17 — 7-DOF 臂 + 2 指 revolute gripper
  {
    {"sciurus17_joint1", "sciurus17_joint2", "sciurus17_joint3",
     "sciurus17_joint4", "sciurus17_joint5", "sciurus17_joint6",
     "sciurus17_joint7"},
    {"sciurus17_left_finger_joint", "sciurus17_right_finger_joint"},
    1.5, 0.0
  },
};

// ============================================================================
// OpenArmXDeployNode — 策略部署 ROS2 节点
// ============================================================================
class OpenArmXDeployNode : public rclcpp::Node {
 public:
  OpenArmXDeployNode();

 private:
  // ---- ROS 回调 ----
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onObjectPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

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
  std::array<double, 3> getCurrentTCP();   // 占位 — 实际需通过 TF/FK
  std::vector<double> solveIK(const std::array<double, 3>& target_pos,
                              const std::array<double, 9>& target_xmat,
                              const std::vector<double>& initial_q);
  void publishJointTrajectory(const std::vector<double>& q);
  void publishGripperCommand(double position);

  // ---- 回初始位姿 ----
  void moveToHome();                    // 发布回初始位姿轨迹并等待到位
  bool waitForHome(double timeout_s);   // 轮询 /joint_states 检查是否到位
  bool planAndExecuteWithMoveIt(const std::vector<double>& target_q);  // MoveIt2 避障规划+执行
  void interpolateToHome();             // 降级方案: 关节空间线性插值 (无避障)

  // ---- 参数 ----
  void loadParameters();

  // ---- ROS 接口 ----
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr grasp_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

  // ---- 模型 ----
  torch::jit::script::Module actor_;

  // ---- 运行时状态 ----
  std::mutex mutex_;
  std::map<std::string, double> arm_qpos_;
  std::map<std::string, double> gripper_qpos_;
  double object_x_ = 0.0, object_y_ = 0.0, object_z_ = 0.0;
  bool joints_received_ = false;
  bool object_received_ = false;

  // ---- 状态机 & 控制 ----
  ObservationBuilder obs_builder_;
  std::unique_ptr<GraspStateMachine> state_machine_;
  int step_count_ = 0;
  bool grasp_active_ = false;
  bool grasp_success_ = false;
  std::thread grasp_thread_;            // 抓取循环线程句柄 (复位时等待退出)
  bool grasp_thread_running_ = false;

  // ---- 初始位姿 (Home) ----
  std::vector<double> home_qpos_;       // 目标初始关节角
  bool home_recorded_ = false;          // 已记录/指定初始位姿
  std::vector<double> home_pose_param_; // 参数指定的固定初始位姿 (空=启动时记录)
  double home_tolerance_ = 0.02;        // 到位容差 (rad)
  double home_timeout_ = 30.0;          // 回位超时 (s)
  int home_interp_steps_ = 50;          // 回位轨迹插值步数

  // ---- MoveIt2 ----
  std::string move_group_name_ = "arm"; // MoveIt 规划组名 (需匹配 moveit 配置)
  bool use_moveit_ = true;              // 回位是否使用 MoveIt2 避障规划
  double moveit_velocity_scale_ = 0.3;  // 回位速度缩放
  double moveit_acceleration_scale_ = 0.3;  // 回位加速度缩放

  // ---- 参数 ----
  int robot_index_ = 0;            // 0=JGZH, 1=OpenArmX, 2=Sciurus17
  std::string model_path_;
  int max_steps_ = 400;
  double action_pos_scale_ = 0.03;
  double action_gripper_scale_ = 0.005;
  double control_rate_ = 30.0;
  double approach_height_ = 0.15;
  double grasp_height_offset_ = 0.008;
  double lift_height_ = 0.12;
  double grasp_yaw_ = 0.0;
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

  // ---- 加载 TorchScript 模型 ----
  try {
    actor_ = torch::jit::load(model_path_);
    RCLCPP_INFO(get_logger(), "Loaded TorchScript model: %s", model_path_.c_str());
  } catch (const c10::Error& e) {
    RCLCPP_FATAL(get_logger(), "Failed to load model: %s", e.what());
    throw;
  }
  actor_.eval();

  // 验证模型: 跑一次 dummy 推理
  {
    auto dummy = torch::randn({1, ObservationBuilder::kObservationDim});
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(dummy);
    auto output = actor_.forward(inputs).toTensor();
    RCLCPP_INFO(get_logger(), "Model test inference: output shape [%ld, %ld]",
                output.size(0), output.size(1));
  }

  const auto& cfg = kRobotConfigs[robot_index_];
  RCLCPP_INFO(get_logger(), "Robot: %zu arm joints, %zu gripper joints",
              cfg.arm_joint_names.size(), cfg.gripper_joint_names.size());
  RCLCPP_INFO(get_logger(), "Observation dim: %d", ObservationBuilder::kObservationDim);

  // ---- 订阅 ----
  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&OpenArmXDeployNode::onJointState, this, std::placeholders::_1));
  object_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/object_pose", 10,
      std::bind(&OpenArmXDeployNode::onObjectPose, this, std::placeholders::_1));

  // ---- 发布 ----
  traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_joint_command", 10);
  gripper_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/gripper_command", 10);

  // ---- 服务 ----
  grasp_srv_ = create_service<std_srvs::srv::Trigger>(
      "/openarmx_grasp",
      std::bind(&OpenArmXDeployNode::handleGrasp, this,
                std::placeholders::_1, std::placeholders::_2));
  reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "/openarmx_reset",
      std::bind(&OpenArmXDeployNode::handleReset, this,
                std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "OpenArmX deploy node started. control_rate=%.1f Hz", control_rate_);
}

// --------------------------------------------------------------------------
// 参数加载
// --------------------------------------------------------------------------
void OpenArmXDeployNode::loadParameters() {
  declare_parameter("robot", "JGZH");
  declare_parameter("checkpoint", "");  // TorchScript .pt model path
  declare_parameter("max_steps", 400);
  declare_parameter("action_pos_scale", 0.03);
  declare_parameter("action_gripper_scale", 0.005);
  declare_parameter("control_rate", 30.0);
  declare_parameter("approach_height", 0.15);
  declare_parameter("grasp_height_offset", 0.008);
  declare_parameter("lift_height", 0.12);
  declare_parameter("grasp_yaw", 0.0);
  declare_parameter("close_steps", 30);
  declare_parameter("settle_steps", 20);
  declare_parameter("lift_steps", 40);

  // ---- 初始位姿参数 ----
  declare_parameter("home_pose", "");  // 固定初始关节角 (逗号分隔, 如 "0.1,-1.2,..."), 空=启动时记录
  declare_parameter("home_tolerance", 0.02);              // 到位容差 (rad)
  declare_parameter("home_timeout", 30.0);                // 回位超时 (s)
  declare_parameter("home_interp_steps", 50);             // 回位轨迹插值步数

  // ---- MoveIt2 参数 ----
  declare_parameter("move_group_name", "arm");           // MoveIt 规划组名
  declare_parameter("use_moveit", true);                 // 回位是否使用 MoveIt2
  declare_parameter("moveit_velocity_scale", 0.3);       // 回位速度缩放 (0~1)
  declare_parameter("moveit_acceleration_scale", 0.3);   // 回位加速度缩放 (0~1)

  auto robot_name = get_parameter("robot").as_string();
  if (robot_name == "JGZH") robot_index_ = 0;
  else if (robot_name == "OpenArmX") robot_index_ = 1;
  else if (robot_name == "Sciurus17") robot_index_ = 2;
  else {
    RCLCPP_WARN(get_logger(), "Unknown robot '%s', using JGZH", robot_name.c_str());
    robot_index_ = 0;
  }

  model_path_ = get_parameter("checkpoint").as_string();
  if (model_path_.empty()) {
    RCLCPP_WARN(get_logger(), "No checkpoint path provided! Use --ros-args -p checkpoint:=<path>");
  }
  max_steps_ = get_parameter("max_steps").as_int();
  action_pos_scale_ = get_parameter("action_pos_scale").as_double();
  action_gripper_scale_ = get_parameter("action_gripper_scale").as_double();
  control_rate_ = get_parameter("control_rate").as_double();
  approach_height_ = get_parameter("approach_height").as_double();
  grasp_height_offset_ = get_parameter("grasp_height_offset").as_double();
  lift_height_ = get_parameter("lift_height").as_double();
  grasp_yaw_ = get_parameter("grasp_yaw").as_double();
  close_steps_ = get_parameter("close_steps").as_int();
  settle_steps_ = get_parameter("settle_steps").as_int();
  lift_steps_ = get_parameter("lift_steps").as_int();

  // ---- 初始位姿 ----
  {
    auto home_pose_str = get_parameter("home_pose").as_string();
    if (!home_pose_str.empty()) {
      // 解析逗号分隔的关节角字符串 -> vector<double>
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

  move_group_name_ = get_parameter("move_group_name").as_string();
  use_moveit_ = get_parameter("use_moveit").as_bool();
  moveit_velocity_scale_ = get_parameter("moveit_velocity_scale").as_double();
  moveit_acceleration_scale_ = get_parameter("moveit_acceleration_scale").as_double();
  if (use_moveit_) {
    RCLCPP_INFO(get_logger(), "MoveIt2 home planning enabled (group='%s', vel_scale=%.2f).",
                move_group_name_.c_str(), moveit_velocity_scale_);
  }

  if (!home_pose_param_.empty()) {
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
  const auto& cfg = kRobotConfigs[robot_index_];
  for (size_t i = 0; i < msg->name.size(); ++i) {
    for (const auto& name : cfg.arm_joint_names) {
      if (msg->name[i] == name) arm_qpos_[name] = msg->position[i];
    }
    for (const auto& name : cfg.gripper_joint_names) {
      if (msg->name[i] == name) gripper_qpos_[name] = msg->position[i];
    }
  }
  joints_received_ = true;

  // 记录初始位姿: 首次收到全部臂关节状态时保存为 home (若参数未指定固定位姿)
  if (!home_recorded_ && home_pose_param_.empty()) {
    bool all_present = true;
    for (const auto& name : cfg.arm_joint_names) {
      if (!arm_qpos_.count(name)) {
        all_present = false;
        break;
      }
    }
    if (all_present) {
      home_qpos_.clear();
      for (const auto& name : cfg.arm_joint_names) {
        home_qpos_.push_back(arm_qpos_[name]);
      }
      home_recorded_ = true;
      RCLCPP_INFO(get_logger(), "Home pose recorded from initial joint states.");
    }
  }
}

void OpenArmXDeployNode::onObjectPose(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  object_x_ = msg->pose.position.x;
  object_y_ = msg->pose.position.y;
  object_z_ = msg->pose.position.z;
  object_received_ = true;
}

// --------------------------------------------------------------------------
// 服务
// --------------------------------------------------------------------------
void OpenArmXDeployNode::handleGrasp(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (grasp_active_) {
    response->success = false;
    response->message = "Grasp already in progress";
    return;
  }
  if (!joints_received_ || !object_received_) {
    response->success = false;
    response->message = "Waiting for /joint_states and /object_pose";
    return;
  }
  // 若上一轮抓取线程仍在收尾 (刚被 reset/自然结束), 先等它退出。
  // 否则重新赋值 std::thread 会对 joinable 线程调用 std::terminate。
  if (grasp_thread_running_ && grasp_thread_.joinable()) {
    grasp_thread_.join();
    grasp_thread_running_ = false;
  }
  grasp_active_ = true;
  grasp_success_ = false;
  response->success = true;
  response->message = "Grasp started";

  // 在独立线程运行抓取循环 (避免阻塞 ROS 回调)
  // 保存线程句柄，复位时等待其退出后再回初始位姿
  grasp_thread_ = std::thread([this]() { runGraspLoop(); });
  grasp_thread_running_ = true;
}

void OpenArmXDeployNode::handleReset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  // ① 停止抓取循环并等待线程退出 (最多一个控制周期)
  grasp_active_ = false;
  if (grasp_thread_running_ && grasp_thread_.joinable()) {
    grasp_thread_.join();
    grasp_thread_running_ = false;
  }

  // ② 回到初始位姿 (带超时保护)
  moveToHome();

  // ③ 清理状态: 销毁状态机, 步数清零, 强制重新感知物体
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
/*
30hz的控制循环：每一步都重新观测(最新的传感器数据)、重新推理(根据当前状态决策)、微调目标并执行动作(实时修正轨迹)
因此相较于传统视觉抓取("视觉定位→逆解→执行")，基于RL伺服的抓取在每一步都能重新规划和动态调整目标，有闭环反馈和实时微调，根据最新的观测和策略输出进行微调，具有更强的鲁棒性和适应性。
*/
void OpenArmXDeployNode::runGraspLoop() {
  RCLCPP_INFO(get_logger(), "Starting grasp loop...");
  step_count_ = 0;

  // 初始化状态机
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::array<double, 3> obj_pos = {object_x_, object_y_, object_z_};
    state_machine_ = std::make_unique<GraspStateMachine>(
        obj_pos, approach_height_, grasp_height_offset_, lift_height_,
        /*table_clearance=*/0.02, grasp_yaw_,
        close_steps_, settle_steps_, lift_steps_,
        kRobotConfigs[robot_index_].gripper_open,
        kRobotConfigs[robot_index_].gripper_close);
  }

  rclcpp::Rate rate(control_rate_);

  while (grasp_active_ && rclcpp::ok()) {
    if (step_count_ >= max_steps_) {
      RCLCPP_WARN(get_logger(), "Max steps (%d) reached. Stopping.", max_steps_);
      break;
    }

    if (state_machine_->isDone()) {
      grasp_success_ = true;
      RCLCPP_INFO(get_logger(), "Grasp completed successfully!");
      break;
    }

    // 1. 构造观测
    auto obs = buildObservation();

    // 2. 策略推理
    auto action = policyAction(obs);

    // 3. 执行动作
    executeAction(action);

    step_count_++;
    rate.sleep();
  }

  grasp_active_ = false;

  if (grasp_success_) {
    RCLCPP_INFO(get_logger(), "Grasp SUCCESS — total steps: %d", step_count_);
  } else {
    RCLCPP_WARN(get_logger(), "Grasp STOPPED — total steps: %d", step_count_);
  }
}

// --------------------------------------------------------------------------
// 观测构造
// --------------------------------------------------------------------------
/*
┌─────────────────────────────────────────────────────────────┐
│ 观测向量 (29维)                                            │
├─────────────────────────────────────────────────────────────┤
│ [0-6]   7个关节角度 (arm_q)                                │
│ [7-8]   2个夹爪位置 (grip_q)                              │
│ [9-11]  物体位置 (ox, oy, oz)                             │
│ [12-14] 手指中心 (近似 = object + 0.15m)                  │
│ [15-17] 物体到手指偏移 (由builder内部计算？)              │
│ [18-20] 目标TCP位置 (target_pos)                          │
│ [21]    目标夹爪开度 (gripper_target)                     │
│ [22]    阶段进度 (step_count/max_steps)                   │
│ [23-28] 阶段one-hot (center_xy/approach/descend/...)     │
└─────────────────────────────────────────────────────────────┘
*/
std::vector<float> OpenArmXDeployNode::buildObservation() {
  const auto& cfg = kRobotConfigs[robot_index_];

  std::vector<double> arm_q;
  std::vector<double> grip_q;
  double ox, oy, oz;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& name : cfg.arm_joint_names) {
      arm_q.push_back(arm_qpos_.count(name) ? arm_qpos_[name] : 0.0);
    }
    for (const auto& name : cfg.gripper_joint_names) {
      grip_q.push_back(gripper_qpos_.count(name) ? gripper_qpos_[name] : 0.0);
    }
    ox = object_x_; oy = object_y_; oz = object_z_;
  }

  obs_builder_.updateArmQpos(arm_q);
  obs_builder_.updateGripperQpos(grip_q);
  obs_builder_.updateObjectPos(ox, oy, oz);

  // fingerpad center 近似 (真实部署用 TF)
  obs_builder_.updateFingerpadCenter(ox, oy, oz + 0.15);

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
/*
输出的是修正量：相对于 expert 目标的增量 ([-1, 1] 映射到实际增量)
输出修正量而不是绝对目标的原因是：策略网络训练时是基于 expert 的轨迹数据，输出修正量可以让策略在 expert 的基础上进行微调，而不是完全依赖策略网络生成绝对目标，这样可以提高学习效率和抓取的稳定性和鲁棒性。
因此在执行动作时，先获取 expert 目标，然后加上策略输出的修正量，得到最终的目标位姿。
策略学习什么？--> 网络学会在什么状态下输出什么样的修正值能获得最大奖励
本质：将复杂任务分解为"粗规划+细调整"
*/
std::vector<float> OpenArmXDeployNode::policyAction(const std::vector<float>& obs) {
  auto tensor = torch::from_blob("视觉定位→逆解→执行"
      const_cast<float*>(obs.data()),
      {1, static_cast<long>(ObservationBuilder::kObservationDim)},
      torch::kFloat32);base_target = state_machine.get_target(object_pos)

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
/*
强化学习策略引导的伺服控制，区别于传统"视觉定位→逆解→执行"方法
*/base_target = state_machine.get_target(object_pos)
void OpenArmXDeployNode::executeAction(const std::vector<float>& action) {
  const auto& cfg = kRobotConfigs[robot_index_];

  std::vector<double> arm_q;
  double ox, oy, oz;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& name : cfg.arm_joint_names) {
      arm_q.push_back(arm_qpos_.count(name) ? arm_qpos_[name] : 0.0);
    }
    ox = object_x_; oy = object_y_; oz = object_z_;
  }

  // — 目标位姿（expert + 策略修正) —
  std::array<double, 3> tar_pos, obj_arr = {ox, oy, oz};
  std::array<double, 9> tar_xmat;
  double expert_gripper;
  state_machine_->getTarget(obj_arr, tar_pos, tar_xmat, expert_gripper);

  tar_pos[0] += action[0] * action_pos_scale_;
  tar_pos[1] += action[1] * action_pos_scale_;
  tar_pos[2] += action[2] * action_pos_scale_;

  // — 夹爪目标 —
  double grip_target = std::clamp(
      expert_gripper + action[3] * action_gripper_scale_,
      cfg.gripper_close, cfg.gripper_open);

  // — 状态机更新 —
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

  // — 发布夹爪指令 —
  publishGripperCommand(grip_target);

  // — IK → 关节指令 —
  auto ik_q = solveIK(tar_pos, tar_xmat, arm_q);
  if (!ik_q.empty()) {
    publishJointTrajectory(ik_q);
  }
}

// --------------------------------------------------------------------------
// 辅助 — TCP / IK / 发布
// --------------------------------------------------------------------------
std::array<double, 3> OpenArmXDeployNode::getCurrentTCP() {
  // 占位：实际部署需通过 TF 或正运动学获取
  std::lock_guard<std::mutex> lock(mutex_);
  return {object_x_, object_y_, object_z_ + 0.15};
}

std::vector<double> OpenArmXDeployNode::solveIK(
    const std::array<double, 3>& /*target_pos*/,
    const std::array<double, 9>& /*target_xmat*/,
    const std::vector<double>& initial_q) {
  // 占位：实际部署需接入 MoveIt2 IK 服务 或 TRAC-IK
  // 示例：
  //   auto client = create_client<...>("/compute_ik");
  //   ... 发送请求 ...
  //   return solution;
  RCLCPP_DEBUG(get_logger(), "IK placeholder — replace with MoveIt2 / TRAC-IK call");
  return initial_q;
}

void OpenArmXDeployNode::publishJointTrajectory(const std::vector<double>& q) {
  auto msg = trajectory_msgs::msg::JointTrajectory();
  msg.header.stamp = now();
  msg.joint_names = kRobotConfigs[robot_index_].arm_joint_names;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = q;
  point.time_from_start = rclcpp::Duration::from_seconds(1.0 / control_rate_);
  msg.points.push_back(point);

  traj_pub_->publish(msg);
}

void OpenArmXDeployNode::publishGripperCommand(double position) {
  auto msg = std_msgs::msg::Float64MultiArray();
  size_t n = kRobotConfigs[robot_index_].gripper_joint_names.size();
  msg.data.resize(n, position);
  gripper_pub_->publish(msg);
}

// --------------------------------------------------------------------------
// 回初始位姿
// --------------------------------------------------------------------------
void OpenArmXDeployNode::moveToHome() {
  const auto& cfg = kRobotConfigs[robot_index_];

  if (!home_recorded_) {
    RCLCPP_WARN(get_logger(),
                "Home pose not available (no joint states yet / no home_pose param). Skipping.");
    return;
  }
  if (home_qpos_.size() != cfg.arm_joint_names.size()) {
    RCLCPP_WARN(get_logger(), "Home pose size (%zu) != arm joint count (%zu). Skipping.",
                home_qpos_.size(), cfg.arm_joint_names.size());
    return;
  }

  // 当前关节角
  std::vector<double> current;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& name : cfg.arm_joint_names) {
      current.push_back(arm_qpos_.count(name) ? arm_qpos_[name] : 0.0);
    }
  }

  // 若已基本在初始位姿, 直接返回
  double max_err = 0.0;
  for (size_t i = 0; i < current.size(); ++i) {
    max_err = std::max(max_err, std::abs(current[i] - home_qpos_[i]));
  }
  if (max_err < home_tolerance_) {
    RCLCPP_INFO(get_logger(), "Already at home pose (max_err=%.4f rad).", max_err);
    return;
  }

  // 优先使用 MoveIt2 避障规划 (带障碍物感知), 失败时降级为关节插值
  bool executed = false;
  if (use_moveit_) {
    executed = planAndExecuteWithMoveIt(home_qpos_);
  }
  if (!executed) {
    RCLCPP_WARN(get_logger(), "MoveIt2 unavailable or failed — falling back to joint-space interpolation (NO obstacle avoidance).");
    interpolateToHome();
  }

  // 等待实际到位
  if (waitForHome(home_timeout_)) {
    RCLCPP_INFO(get_logger(), "Reached home pose.");
  } else {
    RCLCPP_WARN(get_logger(), "Home move timed out after %.1f s.", home_timeout_);
  }
}

// --------------------------------------------------------------------------
// MoveIt2 避障规划 + 执行 (回位)
// --------------------------------------------------------------------------
bool OpenArmXDeployNode::planAndExecuteWithMoveIt(const std::vector<double>& target_q) {
  try {
    // 传入 shared_from_this()，MoveGroupInterface 使用本节点通信
    moveit::planning_interface::MoveGroupInterface move_group(
        shared_from_this(), move_group_name_);

    // 从当前真实关节状态开始规划
    move_group.setStartStateToCurrentState();
    // 关节空间目标 = home 位姿
    move_group.setJointValueTarget(target_q);
    // 回位动作放慢，降低风险
    move_group.setMaxVelocityScalingFactor(moveit_velocity_scale_);
    move_group.setMaxAccelerationScalingFactor(moveit_acceleration_scale_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_result = move_group.plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "MoveIt2 plan failed (error code %d).",
                  static_cast<int>(plan_result.val));
      return false;
    }

    size_t n_pts = plan.trajectory_.joint_trajectory.points.size();
    RCLCPP_INFO(get_logger(), "MoveIt2 plan OK (%zu waypoints, %.2f s). Executing...",
                n_pts,
                n_pts > 0
                    ? plan.trajectory_.joint_trajectory.points.back().time_from_start.sec +
                          plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec / 1e9
                    : 0.0);

    auto exec_result = move_group.execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "MoveIt2 execute failed (error code %d).",
                  static_cast<int>(exec_result.val));
      return false;
    }
    RCLCPP_INFO(get_logger(), "MoveIt2 executed home motion successfully.");
    return true;
  } catch (const std::exception& e) {
    // 例如 move_group 节点未启动、规划组名错误、TF 缺失等
    RCLCPP_WARN(get_logger(), "MoveIt2 exception: %s", e.what());
    return false;
  }
}

// --------------------------------------------------------------------------
// 降级方案: 关节空间线性插值 (无避障)
// --------------------------------------------------------------------------
void OpenArmXDeployNode::interpolateToHome() {
  const auto& cfg = kRobotConfigs[robot_index_];

  std::vector<double> current;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& name : cfg.arm_joint_names) {
      current.push_back(arm_qpos_.count(name) ? arm_qpos_[name] : 0.0);
    }
  }

  // 生成从当前 → home 的多点插值轨迹 (避免一步到位导致关节速度过大)
  auto msg = trajectory_msgs::msg::JointTrajectory();
  msg.header.stamp = now();
  msg.joint_names = cfg.arm_joint_names;
  double step_duration = 1.0 / control_rate_;
  for (int i = 1; i <= home_interp_steps_; ++i) {
    double t = static_cast<double>(i) / home_interp_steps_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.reserve(current.size());
    for (size_t j = 0; j < current.size(); ++j) {
      point.positions.push_back(current[j] + t * (home_qpos_[j] - current[j]));
    }
    point.time_from_start = rclcpp::Duration::from_seconds(i * step_duration);
    msg.points.push_back(point);
  }
  traj_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "Interpolating to home pose (%d steps)...", home_interp_steps_);
}

bool OpenArmXDeployNode::waitForHome(double timeout_s) {
  const auto& cfg = kRobotConfigs[robot_index_];
  if (home_qpos_.size() != cfg.arm_joint_names.size()) return false;
getCurrentTCP
  rclcpp::Time start = now();
  while (rclcpp::ok()) {
    double max_err = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (size_t i = 0; i < cfg.arm_joint_names.size(); ++i) {
        const auto& name = cfg.arm_joint_names[i];
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
    // 多线程 executor: 服务回调 (reset → MoveIt2 plan/execute 阻塞等待 action 结果)
    // 与话题回调 / action 回调并发处理，避免单线程 spin 死锁
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::executors::MultiThreadedExecutor::make_params().num_threads(4));
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("openarmx_deploy"), "Fatal: %s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
