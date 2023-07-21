ROS_DISTRO=foxy
export ROS_DOMAIN_ID=1
#export ROS_LOCALHOST_ONLY=1
#export CYCLONEDDS_URI=file:///etc/cyclonedds/config.xml # cyclonedds 的配置文件，没有可以不写
#export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # 可选rmw_cyclonedds_cpp / rmw_fastrtps_cpp

ENVFILE=/root/aid_ros_ws/install/setup.bash # ros2的环境

if [ -f ${ENVFILE} ] ; then
  echo "Loading robot env..."
  source /opt/ros/${ROS_DISTRO}/setup.bash
  source ${ENVFILE}
  #nohup ros2 run aid_robot_py launch_manager_node &
  ros2 launch robot_bringup robot_bringup.launch.xml
else
  echo "ros robot ws not found"
fi
