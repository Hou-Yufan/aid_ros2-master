/******************************************************************************
 * Copyright (c) 2023 dongfang
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of laser_undistortion nor the
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

#include <dirent.h>

#include <fstream>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
// If you use debug mode and visualize point cloud, you need to install PCL
#define debug_ 0
using namespace std::placeholders;

#if debug_
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
pcl::visualization::CloudViewer g_PointCloudView(
    "PointCloud View");  // Initialize a pcl window
#endif

class LidarMotionCalibrator : public rclcpp::Node {
 private:
  // Declare TF listener, rclcpp handle, scan subscriber, scan publisher

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  sensor_msgs::msg::PointCloud2 laserCloud_;
  std::string scan_frame_name_;
  std::string odom_name_;
  std::string scan_sub_name_;
  std::string scan_pub_name_;
  std::string scanCloudTopicName_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

#if debug_
  // Visualize point cloud objects
  pcl::PointCloud<pcl::PointXYZRGB> visual_cloud_;
#endif
 public:
  // Constructor, initialize tf_listener_, subscriber, callback function
  // ScanCallBack
  LidarMotionCalibrator() : Node("laser_undistortion_node") {

    scanCloudTopicName_ = this->declare_parameter<std::string>(
        "laser_cloud_topic_name", "laser_motion_undistortion");
    scan_pub_name_ = this->declare_parameter<std::string>("scan_pub_name",
                                                          "scan_undistortion");
    scan_sub_name_ =
        this->declare_parameter<std::string>("scan_sub_name", "scan");
    odom_name_ = this->declare_parameter<std::string>("odom_name", "odom");

    rclcpp::QoS qos(rclcpp::KeepLast(20));
    qos.best_effort();
    //qos.reliable();
    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_sub_name_, sensor_qos,
        std::bind(&LidarMotionCalibrator::ScanCallBack, this, _1));

    scan_pub_ =
        this->create_publisher<sensor_msgs::msg::LaserScan>(scan_pub_name_, sensor_qos);
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        scanCloudTopicName_, 2);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
  // Destructor, release tf_listener_
  ~LidarMotionCalibrator() {}
  // Get the original laser data for processing
  void ScanCallBack(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    // Transform to the data needed for correction
    rclcpp::Time startTime, endTime;
    // A frame of scan data comes first, the timestamp at the beginning and end,
    // the size of the data
    startTime = scan_msg->header.stamp;
    sensor_msgs::msg::LaserScan laserScanMsg = *scan_msg;
    // Time to get the final point
    int beamNum = laserScanMsg.ranges.size();
    endTime =
        startTime + rclcpp::Duration(laserScanMsg.time_increment * beamNum * 1000000000);
    scan_frame_name_ = laserScanMsg.header.frame_id;
    // Copy the data
    std::vector<double> angles, ranges;
    //for (int i = beamNum - 1; i >= 0; i--)  
         // This is related to the original lidar rotation direction, if
               // the rotation direction is different, it is changed to the next
               // line of code s1 radar A1 radar test this line of code is
               // normal
     for(int i = 0; i <beamNum ;i++)
    {
      double lidar_dist = laserScanMsg.ranges[i];  // Unit meter
      double lidar_angle = laserScanMsg.angle_min +
                           laserScanMsg.angle_increment * i;  // Unit radian

      ranges.push_back(lidar_dist);
      angles.push_back(lidar_angle);
    }
#if debug_
    visual_cloud_.clear();
    // Convert to pcl::pointcloud for visuailization
    // Before data correction, point cloud visualization, red
    visual_cloud_scan(ranges, angles, 255, 0, 0);
#endif
    // Correct
    Lidar_Calibration(ranges, angles, startTime, endTime);
// After data correction, the packaging plan is to point cloud visualization,
// green Convert to pcl::pointcloud for visuailization
#if debug_
    visual_cloud_scan(ranges, angles, 0, 255, 0);

#endif
    // Publish the corrected scan
    // ROS_INFO("scan_time:%f",rclcpp::Duration(laserScanMsg.time_increment *
    // beamNum).seconds());
    scan_cal_pub(ranges, angles, laserScanMsg);
// Display
#if debug_
    // scan_cloud_pub(startTime);
    g_PointCloudView.showCloud(visual_cloud_.makeShared());
#endif
  }

  // The release function of the scanned data after correction. Here, all data
  // of all 360 degrees are released by default. The installation is in
  // progress. The parameter filled in is rplidar A1
  void scan_cal_pub(const std::vector<double>& ranges,
                    const std::vector<double>& angles,
                    sensor_msgs::msg::LaserScan& sourceScan) {
    // Define sensor_msgs::msg::LaserScan data
    sensor_msgs::msg::LaserScan tempScan;
    tempScan = sourceScan;
    tempScan.angle_increment = (tempScan.angle_max - tempScan.angle_min) /
                               (ranges.size() * 3 - 1);  // This should be noted
    // tempScan.scan_time=(float)scan_time; //Unit s
    // tempScan.time_increment=scan_time/(ranges.size()*3-1); //this is changed
    tempScan.scan_time =
        0.00002;  // Unit s This data can be regarded as instantaneous laser
                  // data, the acquisition time is 0
    tempScan.time_increment =
        tempScan.scan_time /
        (ranges.size() * 3 - 1);  // This data can be regarded as instantaneous
                                  // laser data, the acquisition time is 0

    tempScan.ranges.clear();
    tempScan.intensities.clear();
    tempScan.ranges.resize(
        ranges.size() *
        3);  // The amount of data is increased to three times the original,
             // which is equivalent to the resolution of the angle increment is
             // increased by three times to improve the accuracy of motion
             // distortion correction
    tempScan.intensities.resize(ranges.size() * 3);

    // Fill the radar data to judge whether the filling is correct
    for (size_t i = 0; i < ranges.size(); ++i) {
      int index =
          (angles[i] - tempScan.angle_min) / tempScan.angle_increment + 0.5;
      if (index > ranges.size() * 3 - 1 || index < 0) continue;
      tempScan.ranges[index] = ranges[i];
      tempScan.intensities[index] = 15.0;  // This may be useless
    }
//====================================================
//===========================
#if debug_
    // Visual test of package data
    std::vector<double> angles_temp, ranges_temp;
    for (int i = 0; i < tempScan.ranges.size(); i++) {
      double lidar_dist = tempScan.ranges[i];
      double lidar_angle = tempScan.angle_min + tempScan.angle_increment * i;

      ranges_temp.push_back(lidar_dist);
      angles_temp.push_back(lidar_angle);
    }
// visual_cloud_scan(ranges_temp,angles_temp,255,255,255);
#endif
    //====================================================
    //=========================== release
    scan_pub_->publish(tempScan);
  }

#if debug_
  void scan_cloud_pub(rclcpp::Time pubTime) {
    pcl::toROSMsg(visual_cloud_, laserCloud_);
    laserCloud_.header.frame_id = scan_frame_name_;
    laserCloud_.header.stamp = pubTime;
    cloud_pub_.publish(laserCloud_);
  }
#endif
// Use point cloud to visualize the laser
#if debug_
  void visual_cloud_scan(const std::vector<double>& ranges_,
                         const std::vector<double>& angles_, unsigned char r_,
                         unsigned char g_, unsigned char b_) {
    unsigned char r = r_, g = g_, b = b_;  // Do not rename the variable
    for (int i = 0; i < ranges_.size(); i++) {
      if (ranges_[i] < 0.05 || std::isnan(ranges_[i]) || std::isinf(ranges_[i]))
        continue;

      pcl::PointXYZRGB pt;
      pt.x = ranges_[i] * cos(angles_[i]);
      pt.y = ranges_[i] * sin(angles_[i]);
      pt.z = 0.0;

      // pack r/g/b into rgb
      unsigned int rgb =
          ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
      pt.rgb = *reinterpret_cast<float*>(&rgb);

      visual_cloud_.push_back(pt);
    }
  }
#endif
  /**
   * @name getLaserPose()
   * @brief gets the robot's pose in the odometer coordinate system tf::Pose
   * Get the pose of lidar in odom coordinate system at time dt odom_pose
   * @param odom_pos the pose of the robot
   * @param dt dt moment
   * @param tf_listener_
   */
  bool getLaserPose(tf2::Transform& odom_pose, rclcpp::Time dt) {
    try {
      // tf2::Transform transform;
      geometry_msgs::msg::TransformStamped transform =
          tf_buffer_->lookupTransform(odom_name_, scan_frame_name_, dt,
                                      tf2::durationFromSec(0.1));
      tf2::Vector3 t;
      t.setX(transform.transform.translation.x);
      t.setY(transform.transform.translation.y);
      t.setZ(transform.transform.translation.z);
      odom_pose.setOrigin(t);
      tf2::Quaternion r;
      r.setX(transform.transform.rotation.x);
      r.setY(transform.transform.rotation.y);
      r.setZ(transform.transform.rotation.z);
      r.setW(transform.transform.rotation.w);
      odom_pose.setRotation(r);

      return true;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("laser_undistortion"),
                          "Error getting transform: " << ex.what());
      //throw;
    }

    return true;
  }
  tf2Scalar getYawFromQuaternion(const tf2::Quaternion& quat) {
    tf2::Matrix3x3 matrix(quat);
    tf2Scalar yaw, pitch, roll;
    matrix.getEulerYPR(yaw, pitch, roll);
    return yaw;
  }
  /**
   * @brief Lidar_MotionCalibration
   * At the segmented moment, lidar motion distortion is removed;
   * In this piecewise function, the robot is considered to be moving at a
   * constant speed;
   * @param frame_base_pose datum coordinate system after calibration
   * @param frame_start_pose The pose corresponding to the first laser point in
   * this segment
   * @param frame_end_pose The pose corresponding to the last laser point in
   * this segment
   * @param ranges Laser data-distance
   * @param angles laser data-angle
   * @param startIndex The subscript of the first laser point in this segment in
   * the laser frame
   * @param beam_number The number of laser points in this segment
   */
  void Lidar_MotionCalibration(
      tf2::Transform frame_base_pose,  // For each frame scan, the reference
                                       // coordinate system is consistent
      tf2::Transform frame_start_pose, tf2::Transform frame_end_pose,
      std::vector<double>& ranges, std::vector<double>& angles,
      int startIndex,  // The starting number of the laser point of each segment
      int& beam_number)  // The number of laser points in this segment
  {
    // Step size for linear interpolation for each pose
    double beam_step = 1.0 / (beam_number - 1);
    // The starting angle and the final angle of the robot, expressed in
    // quaternion
    tf2::Quaternion start_angle_q = frame_start_pose.getRotation();
    tf2::Quaternion end_angle_q = frame_end_pose.getRotation();

    // Convert to radians
    double start_angle_r = getYawFromQuaternion(start_angle_q);
    double base_angle_r = getYawFromQuaternion(frame_base_pose.getRotation());

    // The starting pose of the robot
    tf2::Vector3 start_pos = frame_start_pose.getOrigin();  // Ps
    start_pos.setZ(0);

    // Final pose
    tf2::Vector3 end_pos = frame_end_pose.getOrigin();  // Pe
    end_pos.setZ(0);

    // Basic coordinate system
    tf2::Vector3 base_pos = frame_base_pose.getOrigin();
    base_pos.setZ(0);

    double mid_angle;
    tf2::Vector3 mid_pos;
    tf2::Vector3 mid_point;

    double lidar_angle, lidar_dist;
    // The interpolation calculates the pose corresponding to each point
    for (int i = 0; i < beam_number; i++) {
      // Get the angle interpolation of the laser point, linear interpolation
      // requires step size, start and end data
      mid_angle =
          getYawFromQuaternion(start_angle_q.slerp(end_angle_q, beam_step * i));

      // Get the linear interpolation of the odometer pose of the laser point
      mid_pos = start_pos.lerp(end_pos, beam_step * i);

      // Get the coordinates of the laser point in the odom coordinate system
      double tmp_angle;

      // If the lidar is not equal to infinity, you need to correct it.//First
      // read the data to judge
      if (tf2FuzzyZero(ranges[startIndex + i]) == false) {
        // Calculate the coordinates of the corresponding laser point in the
        // odom coordinate system

        // Get the distance and angle of this frame laser beam
        lidar_dist = ranges[startIndex + i];
        lidar_angle = angles[startIndex + i];

        // The coordinates of the laser point in the lidar coordinate system
        double laser_x, laser_y;
        laser_x = lidar_dist * cos(lidar_angle);
        laser_y = lidar_dist * sin(lidar_angle);

        // The coordinates of the laser point under the corresponding odometer
        // coordinate system
        double odom_x, odom_y;
        odom_x =
            laser_x * cos(mid_angle) - laser_y * sin(mid_angle) + mid_pos.x();
        odom_y =
            laser_x * sin(mid_angle) + laser_y * cos(mid_angle) + mid_pos.y();

        // Convert to type
        mid_point.setValue(odom_x, odom_y, 0);
        // Get the coordinates of the laser point in the reference coordinate
        // system Convert the laser data points in the odom coordinate system to
        // the basic coordinate system Get the laser point data that should be
        // measured at that instant
        double x0, y0, a0, s, c;
        x0 = base_pos.x();
        y0 = base_pos.y();
        a0 = base_angle_r;
        s = sin(a0);
        c = cos(a0);
        /*
         * Convert base to odom to [c -s x0;
         * s c y0;
         * 0 0 1]
         * Convert odom to base as [c s -x0*c-y0*s;
         * -s c x0*s-y0*c;
         * 0 0 1]
         */
        double tmp_x, tmp_y;
        tmp_x = mid_point.x() * c + mid_point.y() * s - x0 * c - y0 * s;
        tmp_y = -mid_point.x() * s + mid_point.y() * c + x0 * s - y0 * c;
        mid_point.setValue(tmp_x, tmp_y, 0);
        // Then calculate the dist angle of the laser point starting from the
        // starting coordinate
        double dx, dy;
        dx = (mid_point.x());
        dy = (mid_point.y());
        lidar_dist = sqrt(dx * dx + dy * dy);
        lidar_angle = atan2(dy, dx);

        // Lidar is corrected
        ranges[startIndex + i] = lidar_dist;
        angles[startIndex + i] = lidar_angle;
      }
      // If equal to infinity, then calculate the angle
      else {
        // Laser angle
        lidar_angle = angles[startIndex + i];

        // The angle of the odometer coordinate system
        tmp_angle = mid_angle + lidar_angle;
        tmp_angle = tf2NormalizeAngle(tmp_angle);

        // If the data is illegal, you only need to set the angle. Convert angle
        // to angle in start_pos coordinate system
        lidar_angle = tf2NormalizeAngle(tmp_angle - start_angle_r);

        angles[startIndex + i] = lidar_angle;
      }
    }
  }

  // Lidar data 　 segmented linear interpolation 　 segmented period is 5ms,
  // can be changed Lidar_MotionCalibration() will be called here
  /**
   * @name Lidar_Calibration()
   * @brief lidar data 　 piecewise linear difference 　 segmentation period is
   * 5ms
   * @param ranges laser beam distance value set
   * @param angle　 laser beam angle value set
   * @param startTime　Timestamp of the first laser
   * @param endTime　Timestamp of the last laser
   * @param *tf_listener_
   */
  void Lidar_Calibration(std::vector<double>& ranges,
                         std::vector<double>& angles, rclcpp::Time startTime,
                         rclcpp::Time endTime) {
    // Count the number of laser beams
    int beamNumber = ranges.size();
    if (beamNumber != angles.size()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("laser_undistortion"),
                          "Error:ranges not match to the angles");
      return;
    }

    // 5000us to segment
    int64_t interpolation_time_duration = 50 * 1000 * 1000;  // unit us

    tf2::Transform frame_base_pose;
    tf2::Transform frame_start_pose;
    tf2::Transform frame_mid_pose;
    tf2::Transform frame_end_pose;

    // Start time us
    int64_t start_time = startTime.nanoseconds();  // The conversion unit is us
    int64_t end_time = endTime.nanoseconds();
    int64_t time_inc = (end_time - start_time) /
                       beamNumber;  // time interval of each laser data, unit us

    // The starting coordinates of the currently interpolated segment
    int start_index = 0;

    // The pose of the starting point The reason for getting the starting point
    // here is that the starting point is our base_pose The reference pose of
    // all laser points will be changed to our base_pose

    // Get the pose of lidar in odom coordinate system at time t
    // frame_start_pose, frame_end_pose
    if (!getLaserPose(frame_start_pose, rclcpp::Time(start_time))) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("laser_undistortion"),
                         "Not Start Pose,Can not Calib");
      return;
    }

    if (!getLaserPose(frame_end_pose, rclcpp::Time(end_time))) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("laser_undistortion"),
                         "Not End Pose, Can not Calib");
      return;
    }
    // Count error report
    int cnt = 0;
    // The reference coordinate is the coordinate of the first pose
    frame_base_pose = frame_start_pose;
    for (int i = 0; i < beamNumber; i++) {
      // Piecewise linear, the size of the time period is
      // interpolation_time_duration=5000us
      int64_t mid_time =
          start_time +
          time_inc * (i - start_index);  // mid_time and start_time here are
                                         // reused multiple times
      if (mid_time - start_time > interpolation_time_duration ||
          (i == beamNumber - 1)) {
        cnt++;
        // Get the posture of the temporary end frame_mid_pose in the odometer,
        // corresponding to a laser beam
        if (!getLaserPose(frame_mid_pose, rclcpp::Time(mid_time))) {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("laser_undistortion"),
                              "Mid Pose Error");
          return;
        }
        // Interpolate the current starting point and ending point
        // The number of points in the interpolation_time_duration segment
        // interval, count the beginning and end of the segment interval
        int interp_count = i - start_index +
                           1;  // You can try to calculate, there will usually
                               // be dozens or hundreds of laser points
        // Remove the motion distortion of the laser spot in this segment
        Lidar_MotionCalibration(frame_base_pose, frame_start_pose,
                                frame_mid_pose, ranges, angles, start_index,
                                interp_count);

        // Update time
        start_time = mid_time;
        start_index = i;  // To facilitate the calculation of the number of
                          // laser points in the segment
        frame_start_pose = frame_mid_pose;
      }
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarMotionCalibrator>());
  rclcpp::shutdown();
  return 0;
}
