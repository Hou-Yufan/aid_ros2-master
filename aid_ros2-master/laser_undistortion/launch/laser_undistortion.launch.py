#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('laser_undistortion'),
        'params',
        'undistortion.yaml'
        )

    run_node = LifecycleNode( name='laser_undistortion_node',
                                package='laser_undistortion',
                                executable='laser_undistortion_node',
                                output='screen',
                                parameters=[config])
    return LaunchDescription([
        run_node
    ])
