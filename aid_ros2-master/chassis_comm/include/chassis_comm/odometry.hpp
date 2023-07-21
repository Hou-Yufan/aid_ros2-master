
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
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#ifndef CHASSIS_COMM_NODE__ODOMETRY_HPP_
#define CHASSIS_COMM_NODE__ODOMETRY_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

namespace robotis {
namespace chassis_comm {
class Odometry {
 public:
  explicit Odometry(std::shared_ptr<rclcpp::Node>& nh,
                    const double wheels_separation, const double wheels_radius);
  virtual ~Odometry() {}
  void OdometryProcess(rclcpp::Time time, std::array<double, 2>& wheel);

 private:
  bool calculate_odometry(const rclcpp::Duration& duration);

  void update_joint_state(std::array<double, 2>& wheel);
  void publish(const rclcpp::Time& now);

  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::JointState>>
      msg_ftr_joint_state_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>>
      msg_ftr_imu_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::JointState, sensor_msgs::msg::Imu>
      SyncPolicyJointStateImu;
  typedef message_filters::Synchronizer<SyncPolicyJointStateImu>
      SynchronizerJointStateImu;

  std::shared_ptr<SynchronizerJointStateImu> joint_state_imu_sync_;

  double wheels_separation_;
  double wheels_radius_;

  std::string frame_id_of_odometry_;
  std::string child_frame_id_of_odometry_;

  std::array<double, 2> diff_joint_positions_;
  double imu_angle_;

  std::array<double, 3> robot_pose_;
  std::array<double, 3> robot_vel_;
};
}  // namespace chassis_comm
}  // namespace robotis
#endif  // CHASSIS_COMM_NODE__ODOMETRY_HPP_
