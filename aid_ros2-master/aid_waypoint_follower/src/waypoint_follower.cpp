// Copyright (c) 2019 Samsung Research America
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

#include "aid_waypoint_follower/waypoint_follower.hpp"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>

namespace aid_waypoint_follower {

WaypointFollower::WaypointFollower()
    : nav2_util::LifecycleNode("WaypointFollower", "", false) {
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("stop_on_failure", false);
  declare_parameter("loop_rate", 20);

  stop_on_failure_ = get_parameter("stop_on_failure").as_bool();
  loop_rate_ = get_parameter("loop_rate").as_int();

  std::vector<std::string> new_args = rclcpp::NodeOptions().arguments();
  new_args.push_back("--ros-args");
  new_args.push_back("-r");
  new_args.push_back(std::string("__node:=") + this->get_name() +
                     "_rclcpp_node");
  new_args.push_back("--");
  client_node_ = std::make_shared<rclcpp::Node>(
      "_", "", rclcpp::NodeOptions().arguments(new_args));

  nav_to_pose_client_ =
      rclcpp_action::create_client<ClientT>(client_node_, "navigate_to_pose");
  navi_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/patrol_path", 1,
      std::bind(&WaypointFollower::NaviPathCallback, this,
                std::placeholders::_1));
  navi_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/base_link_pose", 1,
      std::bind(&WaypointFollower::NaviPoseCallback, this,
                std::placeholders::_1));
  patrol_control_server_ =
      this->create_service<aid_robot_msgs::srv::PatrolControl>(
          "patrol_control",
          std::bind(&WaypointFollower::PatrolControlCallback, this,
                    std::placeholders::_1, std::placeholders::_2));

  status_pub_ =
        this->create_publisher<std_msgs::msg::String>("patrol_status", 10);

}

WaypointFollower::~WaypointFollower() {
  RCLCPP_INFO(get_logger(), "Destroying");
}

  void WaypointFollower::TimerCallback() {  // RCLCPP_INFO(this->get_logger(), "Timer callback");
    std_msgs::msg::String status;
    if(is_pause_ == true)
    {
      status.data = "pause";
    } else {
      status.data = "running";
    }
    
    status_pub_->publish(status);
  }

nav2_util::CallbackReturn WaypointFollower::on_configure(
    const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_INFO(get_logger(), "Configuring");
  timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&WaypointFollower::TimerCallback, this));
  return nav2_util::CallbackReturn::SUCCESS;
}

void WaypointFollower::PatrolControlCallback(
    const std::shared_ptr<aid_robot_msgs::srv::PatrolControl::Request> request,
    std::shared_ptr<aid_robot_msgs::srv::PatrolControl::Response> response) {
  if (request->cmd == "cancel") {
    auto cancel_future = nav_to_pose_client_->async_cancel_all_goals();
    rclcpp::spin_until_future_complete(client_node_, cancel_future);
    spin_some(client_node_);
    response->success = true;
    new_goal_ = false;
    is_pause_ = true;
  } else if (request->cmd == "pause") {
    auto cancel_future = nav_to_pose_client_->async_cancel_all_goals();
    rclcpp::spin_until_future_complete(client_node_, cancel_future);
    spin_some(client_node_);
    response->success = true;
    new_goal_ = false;
    is_pause_ = true;
  } else if (request->cmd == "resume") {
    new_goal_ = true;
    is_pause_ = false;
  }
  RCLCPP_INFO(get_logger(), "PatrolControlCallback:" + request->cmd);
}

void WaypointFollower::NaviPathCallback(nav_msgs::msg::Path::SharedPtr path) {
  goal_ = *path;
  new_goal_ = true;
  goal_index_ = 0;
  RCLCPP_INFO(get_logger(), "Succeeded get waypoint");
}

void WaypointFollower::NaviPoseCallback(
    geometry_msgs::msg::PoseStamped::SharedPtr pose) {
  followWaypoints();
}

nav2_util::CallbackReturn WaypointFollower::on_activate(
    const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_INFO(get_logger(), "Activating");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn WaypointFollower::on_deactivate(
    const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn WaypointFollower::on_cleanup(
    const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  nav_to_pose_client_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn WaypointFollower::on_shutdown(
    const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void WaypointFollower::followWaypoints() {
  if (goal_.poses.size() == 0 || is_pause_ == true) {
    return;
  }
  // Check if we need to send a new goal_
  if (new_goal_) {
    new_goal_ = false;
    ClientT::Goal client_goal;
    client_goal.pose = goal_.poses[goal_index_];
    geometry_msgs::msg::PoseStamped next_pose;
    if (goal_index_ + 1 == goal_.poses.size()) {
      next_pose = goal_.poses[0];
    } else {
      next_pose = goal_.poses[goal_index_ + 1];
    }
    double angle = 0;
    if (goal_.poses.size() != 1) {
      angle = std::atan2(
          next_pose.pose.position.y - client_goal.pose.pose.position.y,
          next_pose.pose.position.x - client_goal.pose.pose.position.x);
    }

    client_goal.pose.header.stamp = rclcpp::Time();
    client_goal.pose.pose.orientation.w = std::cos(angle / 2.0);
    client_goal.pose.pose.orientation.z = std::sin(angle / 2.0);

    auto send_goal_options = rclcpp_action::Client<ClientT>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(
        &WaypointFollower::resultCallback, this, std::placeholders::_1);
    send_goal_options.goal_response_callback = std::bind(
        &WaypointFollower::goalResponseCallback, this, std::placeholders::_1);
    future_goal_handle_ =
        nav_to_pose_client_->async_send_goal(client_goal, send_goal_options);
    current_goal_status_ = ActionStatus::PROCESSING;
  }

  if (current_goal_status_ == ActionStatus::FAILED) {
    failed_ids_.push_back(goal_index_);

    if (stop_on_failure_) {
      RCLCPP_WARN(get_logger(),
                  "Failed to process waypoint %i in waypoint "
                  "list and stop on failure is enabled."
                  " Terminating action.",
                  goal_index_);
      failed_ids_.clear();
      return;
    } else {
      RCLCPP_INFO(get_logger(),
                  "Failed to process waypoint %i,"
                  " moving to next.",
                  goal_index_);
    }
  } else if (current_goal_status_ == ActionStatus::SUCCEEDED) {
    RCLCPP_INFO(get_logger(),
                "Succeeded processing waypoint %i, "
                "moving to next.",
                goal_index_);
  }

  if (current_goal_status_ != ActionStatus::PROCESSING &&
      current_goal_status_ != ActionStatus::UNKNOWN) {
    // Update server state
    goal_index_++;
    new_goal_ = true;
    if (goal_.poses.size() == 1) {
      goal_.poses.clear();
      new_goal_ = false;
      is_pause_ = true;
      return;
    }
    if (goal_index_ >= goal_.poses.size()) {
      RCLCPP_INFO(get_logger(), "Completed all %i waypoints requested, Rerun.",
                  goal_.poses.size());
      goal_index_ = 0;
      failed_ids_.clear();
    }
  } else {
    RCLCPP_INFO_EXPRESSION(get_logger(),
                           (static_cast<int>(now().seconds()) % 30 == 0),
                           "Processing waypoint %i...", goal_index_);
  }

  rclcpp::spin_some(client_node_);
}

void WaypointFollower::resultCallback(
    const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      current_goal_status_ = ActionStatus::SUCCEEDED;
      return;
    case rclcpp_action::ResultCode::ABORTED:
      current_goal_status_ = ActionStatus::FAILED;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      current_goal_status_ = ActionStatus::FAILED;
      return;
    default:
      current_goal_status_ = ActionStatus::UNKNOWN;
      return;
  }
}

void WaypointFollower::goalResponseCallback(
    std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr>
        future) {
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(
        get_logger(),
        "navigate_to_pose action client failed to send goal_ to server.");
    current_goal_status_ = ActionStatus::FAILED;
  }
}

}  // namespace aid_waypoint_follower
