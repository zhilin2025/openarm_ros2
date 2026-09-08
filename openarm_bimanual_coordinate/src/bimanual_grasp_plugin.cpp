// Copyright 2026 OpenArm contributors
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

// Command-driven grasp welding for the bimanual OpenArm v11 simulation.
//
// Subscribes to std_msgs/String on /bimanual_grasp/grasp_cmd with a JSON
// payload:
//   {"action": "attach", "id": "left",  "link": "openarm_left_hand",
//    "object": "water_bottle"}
//   {"action": "detach", "id": "left"}
// and reports on /bimanual_grasp/grasp_status with:
//   {"event": "attached"|"detached"|"error", "id": ..., "detail": ...}
//
// Physics work is queued and executed on the Gazebo simulation thread.

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace openarm_bimanual_coordinate
{

struct GraspRequest
{
  std::string action;  // attach | detach
  std::string id;      // logical hand id (left / right / ...)
  std::string link;    // robot link name (attach only)
  std::string object;  // world model name (attach only)
};

class BimanualGraspPlugin : public gazebo::ModelPlugin
{
public:
  BimanualGraspPlugin() = default;
  ~BimanualGraspPlugin() override = default;

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();

    ros_node_ = gazebo_ros::Node::Get(sdf);
    cmd_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
      "grasp_cmd", rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) { OnCommand(msg->data); });

    status_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(
      "grasp_status", rclcpp::QoS(10));

    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo &) { OnUpdate(); });

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "BimanualGraspPlugin ready: cmd=/bimanual_grasp/grasp_cmd status=/bimanual_grasp/grasp_status");
  }

private:
  void OnCommand(const std::string & text)
  {
    GraspRequest req;
    try {
      const auto json = nlohmann::json::parse(text);
      req.action = json.at("action").get<std::string>();
      req.id = json.value("id", "hand");
      if (req.action == "attach") {
        req.link = json.at("link").get<std::string>();
        req.object = json.at("object").get<std::string>();
      }
    } catch (const std::exception & e) {
      Report("error", "?", std::string("bad json: ") + e.what());
      return;
    }
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.push_back(req);
  }

  void OnUpdate()
  {
    std::vector<GraspRequest> batch;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      batch.swap(queue_);
    }
    for (const auto & req : batch) {
      if (req.action == "attach") {
        Attach(req);
      } else if (req.action == "detach") {
        Detach(req);
      } else {
        Report("error", req.id, "unknown action " + req.action);
      }
    }
  }

  void Attach(const GraspRequest & req)
  {
    const std::string joint_name = "bimanual_grasp_" + req.id;
    if (model_->GetJoint(joint_name)) {
      Report("attached", req.id, "already attached (re-attach skipped)");
      return;
    }

    auto robot_link = model_->GetLink(req.link);
    auto object = world_->ModelByName(req.object);
    if (!robot_link || !object || object->GetLinks().empty()) {
      Report("error", req.id, "attach failed: link or object not found");
      return;
    }
    auto object_link = object->GetLink();
    if (!object_link) {
      Report("error", req.id, "attach failed: object has no link");
      return;
    }

    auto joint = model_->CreateJoint(joint_name, "fixed", robot_link, object_link);
    if (!joint) {
      Report("error", req.id, "joint creation failed");
      return;
    }
    joint->Init();
    joints_[req.id] = joint_name;
    Report("attached", req.id, req.object);
  }

  void Detach(const GraspRequest & req)
  {
    auto it = joints_.find(req.id);
    if (it == joints_.end()) {
      Report("detached", req.id, "was not attached");
      return;
    }
    if (model_->GetJoint(it->second)) {
      auto joint = model_->GetJoint(it->second);
      joint->Detach();
      model_->RemoveJoint(it->second);
    }
    joints_.erase(it);
    Report("detached", req.id, "");
  }

  void Report(const std::string & event, const std::string & id, const std::string & detail)
  {
    std_msgs::msg::String msg;
    nlohmann::json json;
    json["event"] = event;
    json["id"] = id;
    json["detail"] = detail;
    msg.data = json.dump();
    status_pub_->publish(msg);
    RCLCPP_INFO(ros_node_->get_logger(), "grasp %s id=%s %s",
      event.c_str(), id.c_str(), detail.c_str());
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  gazebo::event::ConnectionPtr update_conn_;

  std::mutex queue_mutex_;
  std::vector<GraspRequest> queue_;
  std::unordered_map<std::string, std::string> joints_;  // id -> joint name
};

GZ_REGISTER_MODEL_PLUGIN(BimanualGraspPlugin)

}  // namespace openarm_bimanual_coordinate
