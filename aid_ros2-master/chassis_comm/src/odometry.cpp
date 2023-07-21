#include "chassis_comm/odometry.hpp"

#include <memory>
#include <string>
#include <utility>

using robotis::chassis_comm::Odometry;
using namespace std::chrono_literals;

Odometry::Odometry(std::shared_ptr<rclcpp::Node>& nh,
                   const double wheels_separation, const double wheels_radius)
    : nh_(nh),
      wheels_separation_(wheels_separation),
      wheels_radius_(wheels_radius),
      imu_angle_(0.0f) {
  RCLCPP_INFO(rclcpp::get_logger("odometry"), "Init Odometry");

  nh_->declare_parameter("odometry.frame_id");
  nh_->declare_parameter("odometry.child_frame_id");

  nh_->get_parameter_or<std::string>("odometry.frame_id", frame_id_of_odometry_,
                                     std::string("odom"));

  nh_->get_parameter_or<std::string>("odometry.child_frame_id",
                                     child_frame_id_of_odometry_,
                                     std::string("base_footprint"));

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("odom", qos);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);
}

void Odometry::OdometryProcess(rclcpp::Time time,
                               std::array<double, 2>& wheel) {
  static rclcpp::Time last_time = time;
  rclcpp::Duration duration(time.nanoseconds() - last_time.nanoseconds());

  update_joint_state(wheel);
  calculate_odometry(duration);
  publish(time);

  last_time = time;
}

void Odometry::publish(const rclcpp::Time& now) {
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

  odom_msg->header.frame_id = frame_id_of_odometry_;
  odom_msg->child_frame_id = child_frame_id_of_odometry_;
  odom_msg->header.stamp = now;

  odom_msg->pose.pose.position.x = robot_pose_[0];
  odom_msg->pose.pose.position.y = robot_pose_[1];
  odom_msg->pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, robot_pose_[2]);

  odom_msg->pose.pose.orientation.x = q.x();
  odom_msg->pose.pose.orientation.y = q.y();
  odom_msg->pose.pose.orientation.z = q.z();
  odom_msg->pose.pose.orientation.w = q.w();

  odom_msg->twist.twist.linear.x = robot_vel_[0];
  odom_msg->twist.twist.angular.z = robot_vel_[2];

  // TODO(Will Son): Find more accurate covariance.
  // odom_msg->pose.covariance[0] = 0.05;
  // odom_msg->pose.covariance[7] = 0.05;
  // odom_msg->pose.covariance[14] = 1.0e-9;
  // odom_msg->pose.covariance[21] = 1.0e-9;
  // odom_msg->pose.covariance[28] = 1.0e-9;
  // odom_msg->pose.covariance[35] = 0.0872665;

  // odom_msg->twist.covariance[0] = 0.001;
  // odom_msg->twist.covariance[7] = 1.0e-9;
  // odom_msg->twist.covariance[14] = 1.0e-9;
  // odom_msg->twist.covariance[21] = 1.0e-9;
  // odom_msg->twist.covariance[28] = 1.0e-9;
  // odom_msg->twist.covariance[35] = 0.001;

  geometry_msgs::msg::TransformStamped odom_tf;

  odom_tf.transform.translation.x = odom_msg->pose.pose.position.x;
  odom_tf.transform.translation.y = odom_msg->pose.pose.position.y;
  odom_tf.transform.translation.z = odom_msg->pose.pose.position.z;
  odom_tf.transform.rotation = odom_msg->pose.pose.orientation;

  odom_tf.header.frame_id = frame_id_of_odometry_;
  odom_tf.child_frame_id = child_frame_id_of_odometry_;
  odom_tf.header.stamp = now;

  odom_pub_->publish(std::move(odom_msg));

  tf_broadcaster_->sendTransform(odom_tf);
}

void Odometry::update_joint_state(std::array<double, 2>& wheel) {
  static std::array<double, 2> last_joint_positions = {0.0f, 0.0f};
  static bool if_first = true;

  diff_joint_positions_[0] = (wheel[0] - last_joint_positions[0]) * 2.0 * M_PI;
  diff_joint_positions_[1] = (wheel[1] - last_joint_positions[1]) * 2.0 * M_PI;

  if (if_first) {
    if_first = false;
    diff_joint_positions_[0] = 0;
    diff_joint_positions_[1] = 0;
  }

  if (fabs(diff_joint_positions_[1]) > 1 ||
      fabs(diff_joint_positions_[1] > 1)) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("odometry"),
                        "diff_joint_positions : " << diff_joint_positions_[0]
                                                  << "  "
                                                  << diff_joint_positions_[1]);
    diff_joint_positions_[0] = 0;
    diff_joint_positions_[1] = 0;
    return;
  }

  last_joint_positions[0] = wheel[0];
  last_joint_positions[1] = wheel[1];
}

bool Odometry::calculate_odometry(const rclcpp::Duration& duration) {
  // rotation value of wheel [rad]
  double wheel_l = diff_joint_positions_[0];
  double wheel_r = diff_joint_positions_[1];

  double delta_s = 0.0;
  double delta_theta = 0.0;

  double theta = 0.0;
  static double last_theta = 0.0;

  // v = translational velocity [m/s]
  // w = rotational velocity [rad/s]
  double v = 0.0;
  double w = 0.0;

  double step_time = duration.seconds();

  if (step_time == 0.0) {
    return false;
  }

  if (std::isnan(wheel_l)) {
    wheel_l = 0.0;
  }

  if (std::isnan(wheel_r)) {
    wheel_r = 0.0;
  }

  delta_s = wheels_radius_ * (wheel_r + wheel_l) / 2.0;

  theta = wheels_radius_ * (wheel_r - wheel_l) / wheels_separation_;
  delta_theta = theta;

  // compute odometric pose
  robot_pose_[0] += delta_s * cos(robot_pose_[2] + (delta_theta / 2.0));
  robot_pose_[1] += delta_s * sin(robot_pose_[2] + (delta_theta / 2.0));
  robot_pose_[2] += delta_theta;

  RCLCPP_DEBUG(rclcpp::get_logger("odometry"), "x : %f, y : %f", robot_pose_[0],
               robot_pose_[1]);

  // compute odometric instantaneouse velocity
  v = delta_s / step_time;
  w = delta_theta / step_time;

  robot_vel_[0] = v;
  robot_vel_[1] = 0.0;
  robot_vel_[2] = w;

  last_theta = theta;
  return true;
}
