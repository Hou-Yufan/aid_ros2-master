################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(robot_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(aid_robot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
# find_package(nav2_map_server)

include_directories(
  include
)

set(DEPENDENCIES
  "rclcpp"
  "geometry_msgs"
  "nav2_msgs"
  "aid_robot_msgs"
  "cartographer_ros_msgs"
  # "nav2_map_server"
)

add_executable(robot_status_manager_node src/robot_status_manager.cpp)
ament_target_dependencies(robot_status_manager_node ${DEPENDENCIES})

add_executable(forbidden_map_create_node src/forbidden_map_create.cpp)
ament_target_dependencies(forbidden_map_create_node ${DEPENDENCIES})

################################################################################
# Install
################################################################################
install(TARGETS robot_status_manager_node forbidden_map_create_node
DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch param script urdf
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
