cmake_minimum_required(VERSION 3.5)
project(laser_undistortion)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

message(status  ${PCL_INCLUDE_DIRS})

set(DEPENDENCIES
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/lidar_motion_undistortion.cpp)

ament_target_dependencies(${PROJECT_NAME}_node ${DEPENDENCIES})

 target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
 )

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})


ament_package()
