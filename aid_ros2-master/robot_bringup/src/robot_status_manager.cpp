/******************************************************************************
 * Copyright (c) 2023 dongfang chen
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of chassis_comm nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include <boost/asio.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "aid_robot_msgs/srv/control_launch.hpp"
#include "aid_robot_msgs/srv/get_current_map.hpp"
#include "aid_robot_msgs/srv/get_string.hpp"
#include "aid_robot_msgs/srv/map_operation.hpp"
#include "aid_robot_msgs/srv/status_change.hpp"
#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"
#include "cartographer_ros_msgs/srv/write_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
// #include "nav2_map_server/map_saver.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class StatusManagerNode : public rclcpp::Node {
 private:
  std::string maping_launch_file =
      "/aid_cartographer/share/aid_cartographer/launch/cartographer.launch.py";
  std::string localization_launch_file =
      "/aid_cartographer/share/aid_cartographer/launch/"
      "localization_2d.launch.py";
  std::string navigation_launch_file =
      "/aid_navigation2/share/aid_navigation2/launch/navigation2.launch.py";
  std::shared_ptr<rclcpp::Node> nh_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string set_status_;
  std::string control_model_;
  std::string slam_status_;
  std::string workspace_path_;
  std::string map_filename_;

  rclcpp::Service<aid_robot_msgs::srv::StatusChange>::SharedPtr
      status_change_server_;
  rclcpp::Service<aid_robot_msgs::srv::MapOperation>::SharedPtr
      map_save_server_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      init_pose_sub_;
  rclcpp::Service<aid_robot_msgs::srv::GetString>::SharedPtr ip_server_;

  bool FinishTrajectory() {
    auto node = rclcpp::Node::make_shared("finish_trajectory_client");
    auto finish_trajectory_client =
        node->create_client<cartographer_ros_msgs::srv::FinishTrajectory>(
            "finish_trajectory");
    auto request = std::make_shared<
        cartographer_ros_msgs::srv::FinishTrajectory::Request>();
    request->trajectory_id = 0;
    if (false == finish_trajectory_client->wait_for_service(2s)) {
      RCLCPP_INFO(
          rclcpp::get_logger("finish_trajectory_client"),
          "Wait finish_trajectory failed,service not available, return ...");
      return false;
    }
    auto result = finish_trajectory_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result,
                                           std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("finish_trajectory_client"),
                         "message:" << result.get()->status.message);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("finish_trajectory_client"),
                   "Failed to call service finish_trajectory");
      return false;
    }
    return true;
  }

  void GetIpHandleRequest(
      const std::shared_ptr<aid_robot_msgs::srv::GetString::Request> request,
      std::shared_ptr<aid_robot_msgs::srv::GetString::Response> response) {
    (void)request;

    std::string ip_address = GetIpAddress();

    if (ip_address.empty()) {
      response->success = false;
      response->message = "Failed to get IP address";
      RCLCPP_ERROR(get_logger(), "Failed to get IP address");
    } else {
      response->success = true;
      response->message = "Success";
      response->result = ip_address;
      RCLCPP_INFO(get_logger(), "IP Address: %s", ip_address.c_str());
    }
  }

  std::string GetIpAddress() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    std::string ip;
    if (sockfd == -1) {
      perror("socket");
      return "";
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, "wlan0", IFNAMSIZ - 1);

    if (ioctl(sockfd, SIOCGIFADDR, &ifr) == -1) {
      perror("ioctl");
      close(sockfd);
      return "";
    }

    close(sockfd);

    struct sockaddr_in *addr = (struct sockaddr_in *)&ifr.ifr_addr;
    char ip_address[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &addr->sin_addr, ip_address, INET_ADDRSTRLEN);
    ip.append(ip_address);
    return ip;
  }

  bool CartographerSaveMap(std::string filename) {
    auto node = rclcpp::Node::make_shared("write_state_client");
    auto write_state_client =
        node->create_client<cartographer_ros_msgs::srv::WriteState>(
            "write_state");
    auto request =
        std::make_shared<cartographer_ros_msgs::srv::WriteState::Request>();
    request->filename = filename + ".pbstream";
    if (false == write_state_client->wait_for_service(2s)) {
      RCLCPP_INFO(rclcpp::get_logger("write_state_client"),
                  "Wait write_state failed, service not available, return ...");
      return false;
    }
    auto result = write_state_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result,
                                           std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("write_state_client"),
                         "message:" << result.get()->status.message);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("write_state_client"),
                   "Failed to call service write_state");
      return false;
    }
    return true;
  }

  bool GetCurrentMap(std::string &filename) {
    auto node = rclcpp::Node::make_shared("get_current_map_client");
    auto get_current_map_client =
        node->create_client<aid_robot_msgs::srv::GetCurrentMap>(
            "get_current_map_id");
    auto request =
        std::make_shared<aid_robot_msgs::srv::GetCurrentMap::Request>();

    if (false == get_current_map_client->wait_for_service(2s)) {
      RCLCPP_INFO(
          rclcpp::get_logger("get_current_map_client"),
          "Wait finish_trajectory failed,service not available, return ...");
      return false;
    }
    auto result = get_current_map_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result,
                                           std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("get_current_map_client"),
                         "message:success");
      filename = result.get()->map_file;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("get_current_map_client"),
                   "Failed to call service get_current_map_idp");
      return false;
    }
    return true;
  }
  // bool MapSaverSaveMap(std::string filename) {
  //   int retcode;
  //   nav2_map_server::SaveParameters save_parameters;
  //   save_parameters.map_file_name = filename;
  //   try {
  //     auto map_saver = std::make_shared<nav2_map_server::MapSaver>();
  //     if (map_saver->saveMapTopicToFile("map", save_parameters)) {
  //       retcode = 0;
  //     } else {
  //       retcode = 1;
  //     }
  //   } catch (std::exception &e) {
  //     RCLCPP_ERROR(rclcpp::get_logger("map_saver_client"),
  //                  "Unexpected problem appear: %s", e.what());
  //     retcode = -1;
  //     return false;
  //   }

  //   return true;
  // }

  bool MapSaverSaveMap2(std::string filename) {
    auto node = rclcpp::Node::make_shared("map_saver_client");
    auto map_saver_client =
        node->create_client<nav2_msgs::srv::SaveMap>("map_saver/save_map");
    auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
    request->map_url = filename;
    if (false == map_saver_client->wait_for_service(2s)) {
      RCLCPP_INFO(
          rclcpp::get_logger("rclcpp"),
          "Wait map_saver/save_map failed,service not available, return ...");
      return false;
    }
    auto result = map_saver_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result,
                                           std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("map_saver_client"),
                         "message:success");

    } else {
      RCLCPP_ERROR(rclcpp::get_logger("map_saver_client"),
                   "Failed to call service /map_saver/save_map");
      return false;
    }
    return true;
  }
  bool MapServerLoadMap(std::string filename) {
    auto node = rclcpp::Node::make_shared("map_server_client");
    auto map_server_client =
        node->create_client<nav2_msgs::srv::LoadMap>("map_server/load_map");
    auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    request->map_url = filename;
    if (false == map_server_client->wait_for_service(2s)) {
      RCLCPP_INFO(
          rclcpp::get_logger("map_server_client"),
          "Wait map_server/load_map failed,service not available, return ...");
      return false;
    }
    auto result = map_server_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result,
                                           std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("map_server_client"),
                         "message:success");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("map_server_client"),
                   "Failed to call service /map_server/load_map");
      return false;
    }
    return true;
  }

  void TimerCallback() {  // RCLCPP_INFO(this->get_logger(), "Timer callback");
    std_msgs::msg::String status;
    status.data = slam_status_ + "+" + control_model_;
    status_pub_->publish(status);
  }
  void CommandVelocityCallback(
      const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg) {
    if (slam_status_ == "mapping" || control_model_ == "remote_control") {
      cmd_vel_pub_->publish(*cmd_vel_msg);
    } else {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager"),
                         "Can not control,only mapping or remote_control can "
                         "control,now status is "
                             << control_model_);
    }
  }

  void InitPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    geometry_msgs::msg::PoseStamped p = *pose;
  }

  void SaveMapCallback(
      const std::shared_ptr<aid_robot_msgs::srv::MapOperation::Request> request,
      std::shared_ptr<aid_robot_msgs::srv::MapOperation::Response> response) {
    if (slam_status_ == "mapping") {
      if (MapSaverSaveMap2(request->map_file_name) == false) {
        response->message = "map saver save map file name failed";
        RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager"),
                           response->message);
        StopLaunch(maping_launch_file);
        response->success = false;
        slam_status_ = "idle";
        return;
      }
      if (FinishTrajectory() == false) {
        response->message = "Finish trajectory failde";
        RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager"),
                           response->message);
        response->success = false;
        StopLaunch(maping_launch_file);
        slam_status_ = "idle";
        return;
      }

      if (CartographerSaveMap(request->map_file_name) == false) {
        response->message = "cartographer save map file name failed";
        RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager"),
                           response->message);
        StopLaunch(maping_launch_file);
        response->success = false;
        slam_status_ = "idle";
        return;
      }

      StopLaunch(maping_launch_file);
      slam_status_ = "idle";
      response->success = true;
      return;
    }

    response->message = "must change mode to mapping first";
    RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager"),
                       response->message);
    response->success = false;
  }

  void ModeSetCallback(
      const std::shared_ptr<aid_robot_msgs::srv::StatusChange::Request> request,
      std::shared_ptr<aid_robot_msgs::srv::StatusChange::Response> response) {
    bool run_status = false;
    set_status_ = request->action;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager"),
                       "set model to" << set_status_);
    if (set_status_ == "mapping") {
      if (slam_status_ == "localization") {
        StopLaunch(localization_launch_file);
      } else {
        StopLaunch(maping_launch_file);
      }

      run_status = StartLaunch(maping_launch_file);
      if (run_status == true) {
        slam_status_ = set_status_;
      }

    } else if (set_status_ == "localization") {
      if (slam_status_ == "mapping") {
        StopLaunch(maping_launch_file);
      } else {
        StopLaunch(localization_launch_file);
      }

      if (GetCurrentMap(map_filename_)) {
        run_status = StartLaunch(localization_launch_file,
                                 "load_state_filename:=" + map_filename_);
        if (run_status == true) {
          slam_status_ = set_status_;
        }
      }
    } else if (set_status_ == "patrol") {
      if (slam_status_ == "localization") {
        run_status = StartLaunch(navigation_launch_file);

        if (run_status == true) {
          control_model_ = set_status_;
        }
      } else {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager"),
                           "can not set to patrol model");
      }
    } else if (set_status_ == "remote_control") {
      StopLaunch(navigation_launch_file);
      control_model_ = set_status_;
      run_status = true;
    } else if (set_status_ == "idle") {
      StopLaunch(navigation_launch_file);
      StopLaunch(maping_launch_file);
      control_model_ = set_status_;
      run_status = true;
    }
    if (run_status)
      response->message = "ok";
    else
      response->message = "err";
    return;
  }

 public:
  StatusManagerNode() : Node("robot_status_manager_node") {
    control_model_ = "idle";
    nh_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
    status_change_server_ =
        nh_->create_service<aid_robot_msgs::srv::StatusChange>(
            "mode_set",
            std::bind(&StatusManagerNode::ModeSetCallback, this, _1, _2));
    ip_server_ = nh_->create_service<aid_robot_msgs::srv::GetString>(
        "get_ip_addresses",
        std::bind(&StatusManagerNode::GetIpHandleRequest, this, _1, _2));
    map_save_server_ = nh_->create_service<aid_robot_msgs::srv::MapOperation>(
        "aid_save_map",
        std::bind(&StatusManagerNode::SaveMapCallback, this, _1, _2));
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&StatusManagerNode::TimerCallback, this));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_remote_ctrl", 10,
        std::bind(&StatusManagerNode::CommandVelocityCallback, this,
                  std::placeholders::_1));
    init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "aid_init_pose", 10,
        std::bind(&StatusManagerNode::InitPoseCallback, this,
                  std::placeholders::_1));
    cmd_vel_pub_ =
        nh_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    status_pub_ =
        nh_->create_publisher<std_msgs::msg::String>("robot_status", 10);

    char *colcon_prefix_path = std::getenv("COLCON_PREFIX_PATH");
    if (colcon_prefix_path == nullptr) {
      RCLCPP_ERROR(rclcpp::get_logger("robot_status_manager_node"),
                   "Unable to get COLCON_PREFIX_PATH environment variable");
    }

    workspace_path_.append(colcon_prefix_path);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager_node"),
                       "COLCON_PREFIX_PATH = " << workspace_path_);
    slam_status_ = "idle";
  }

  bool StartLaunch(std::string file, std::string param = "") {
    auto node = rclcpp::Node::make_shared("start_launch_client");
    auto start_launch_client =
        node->create_client<aid_robot_msgs::srv::ControlLaunch>("start_launch");
    auto request =
        std::make_shared<aid_robot_msgs::srv::ControlLaunch::Request>();
    request->launch_file = workspace_path_ + file;
    request->parameter = param;

    if (false == start_launch_client->wait_for_service(2s)) {
      RCLCPP_INFO(rclcpp::get_logger("start_launch_client"),
                  "service not available");
      return false;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("start_launch_client"),
                       "lanuch file:" << file << " param: " << param);

    auto result = start_launch_client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result,
                                           std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("start_launch_client"),
                         "message:" << result.get()->message);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("start_launch_client"),
                   "Failed to call service start_launch");
      return false;
    }
    return true;
  }
  bool StopLaunch(std::string file) {
    auto node = rclcpp::Node::make_shared("stop_launch_client");
    auto stop_launch_client =
        node->create_client<aid_robot_msgs::srv::ControlLaunch>("stop_launch");

    auto request =
        std::make_shared<aid_robot_msgs::srv::ControlLaunch::Request>();
    request->launch_file = workspace_path_ + file;

    if (!stop_launch_client->wait_for_service(5s)) {
      RCLCPP_INFO(rclcpp::get_logger("stop_launch_client"),
                  "service stop_launch not available");
      return false;
    }

    auto result = stop_launch_client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result,
                                           std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("stop_launch_client"),
                         "message:" << result.get()->message);
      rclcpp::sleep_for(std::chrono::seconds(1));
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("stop_launch_client"),
                   "Failed to call service stop_launch");
      return false;
    }
    return true;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StatusManagerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
