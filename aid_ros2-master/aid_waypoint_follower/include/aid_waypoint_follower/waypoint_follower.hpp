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

#ifndef AID_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_
#define AID_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "aid_robot_msgs/srv/patrol_control.hpp"
#include "std_msgs/msg/string.hpp"

namespace aid_waypoint_follower
{

enum class ActionStatus
{
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3
};

/**
 * @class aid_waypoint_follower::WaypointFollower
 * @brief An action server that uses behavior tree for navigating a robot to its
 * goal position.
 */
class WaypointFollower : public nav2_util::LifecycleNode
{
public:
  using ClientT = nav2_msgs::action::NavigateToPose;
  using ActionClient = rclcpp_action::Client<ClientT>;

  /**
   * @brief A constructor for aid_waypoint_follower::WaypointFollower class
   */
  WaypointFollower();
  /**
   * @brief A destructor for aid_waypoint_follower::WaypointFollower class
   */
  ~WaypointFollower();

protected:
  /**
   * @brief Configures member variables
   *
   * Initializes action server for "FollowWaypoints"
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Action server callbacks
   */
  void followWaypoints();

  /**
   * @brief Action client result callback
   * @param result Result of action server updated asynchronously
   */
  void resultCallback(const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult & result);

  /**
   * @brief Action client goal response callback
   * @param future Shared future to goalhandle
   */
  void goalResponseCallback(
    std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future);
  void TimerCallback();
  void NaviPathCallback(nav_msgs::msg::Path::SharedPtr path);
  void NaviPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr pose);
  void PatrolControlCallback(const std::shared_ptr<aid_robot_msgs::srv::PatrolControl::Request> request,
      std::shared_ptr<aid_robot_msgs::srv::PatrolControl::Response> response);

  // Our action server
  ActionClient::SharedPtr nav_to_pose_client_;
  rclcpp::Node::SharedPtr client_node_;
  std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future_goal_handle_;
  bool stop_on_failure_;
  ActionStatus current_goal_status_;
  int loop_rate_;
  std::vector<int> failed_ids_;
  nav_msgs::msg::Path goal_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr navi_path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr navi_pose_sub_;
  rclcpp::Service<aid_robot_msgs::srv::PatrolControl>::SharedPtr patrol_control_server_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  uint32_t goal_index_ = 0;
  bool new_goal_ = false;
  bool is_pause_ = false;
  rclcpp::Time last_send_goal_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace aid_waypoint_follower

#endif  // AID_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_
