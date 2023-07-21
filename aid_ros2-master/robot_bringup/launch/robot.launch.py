#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():

    LDS_LAUNCH_FILE = '/LDS-50C-C20E_launch.py'
    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
        default=os.path.join(get_package_share_directory('bluesea2'), 'launch'))
    chassis_comm_pkg_dir = LaunchConfiguration(
        'chassis_comm_pkg_dir',
        default=os.path.join(get_package_share_directory('chassis_comm'), 'launch'))
    aid_cartographer_pkg_dir = LaunchConfiguration(
        'aid_cartographer_pkg_dir',
        default=os.path.join(get_package_share_directory('aid_cartographer'), 'launch'))
    laser_undistortion_pkg_dir = LaunchConfiguration(
        'laser_undistortion_pkg_dir',
        default=os.path.join(get_package_share_directory('laser_undistortion'), 'launch'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/robot_state_publisher.launch.py']),
            launch_arguments={}.items(),
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([chassis_comm_pkg_dir, "/chassis_comm.launch.py"]),
        #     launch_arguments={}.items(),
        # ),
        Node(
            package='chassis_comm',
            executable='chassis_comm',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            package='aid_robot_py',
            executable='robot_pose_pub_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            package='aid_robot_py',
            executable='map_manager_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            package='robot_bringup',
            executable='robot_status_manager_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([laser_undistortion_pkg_dir, "/laser_undistortion.launch.py"]),
            launch_arguments={}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={}.items(),
        ),
        Node(
            package='aid_robot_py',
            executable='map_transform_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            package='aid_robot_py',
            executable='launch_manager_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            package='robot_bringup',
            executable='forbidden_map_create_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            package='aid_robot_py',
            executable='ai_manager_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
    ])
