cmake_minimum_required(VERSION 3.5)
project(nrs_waypoint)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(waypoint_publisher src/waypoint_publisher.cpp)
ament_target_dependencies(
  waypoint_publisher
  rclcpp
  std_msgs
)

install(TARGETS
  waypoint_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
