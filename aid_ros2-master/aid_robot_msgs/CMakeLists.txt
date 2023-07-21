cmake_minimum_required(VERSION 3.5)
project(aid_robot_msgs)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StartToEndPoint.msg"
  "srv/QueryLaunchStatus.srv"
  "srv/ControlLaunch.srv"
  "srv/GetCurrentMap.srv"
  "srv/SetCurrentMap.srv"
  "srv/MapImage.srv"
  "srv/MapLinkedDataList.srv"
  "srv/MapList.srv"
  "srv/MapOperationAdd.srv"
  "srv/OperationAdd.srv"
  "srv/OperationGet.srv"
  "srv/ForbiddenGet.srv"
  "srv/OperationDelete.srv"
  "srv/OperationUpdate.srv"
  "srv/MapOperation.srv"
  "srv/StatusChange.srv"
  "srv/PatrolControl.srv"
  "srv/DrawPicture.srv"
  "srv/AICmd.srv"
  "srv/GetCurrentForbidden.srv"
  "srv/GetString.srv"
  DEPENDENCIES nav_msgs
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime nav2_msgs nav_msgs geometry_msgs)

ament_package()
