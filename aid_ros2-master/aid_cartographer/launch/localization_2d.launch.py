import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    aid_cartographer_prefix = get_package_share_directory('aid_cartographer')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  aid_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='aid_localization.lua')
    load_state_filename = LaunchConfiguration('load_state_filename',
                                                 default='/root/maps/1680093350760')
    occupancy_grid_topic = LaunchConfiguration('occupancy_grid_topic',
                                                 default='/global_costmap/localization_map')

    rviz_config_dir = os.path.join(get_package_share_directory('aid_cartographer'),
                                   'rviz', 'aid_cartographer.rviz')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='5.0')
    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'load_state_filename',
            default_value=load_state_filename,
            description='load state filename,load map'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {"yaml_filename":"/opt/ros/foxy/share/nav2_bringup/maps/turtlebot3_world.yaml"}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename,
                       '-load_state_filename', [load_state_filename,".pbstream"],
                       '--ros-args', '--log-level','WARN'],
            remappings = [('scan', 'scan_undistortion')],),
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution, 'occupancy_grid_topic': occupancy_grid_topic,
                              'publish_period_sec': publish_period_sec}.items(),
        ),

    ])
