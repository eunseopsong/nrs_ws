cmake_minimum_required(VERSION 3.5)
project(nrs_waypoint)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(clicked_point_publisher src/clicked_point_publisher.cpp)

ament_target_dependencies(
  clicked_point_publisher
  rclcpp
  geometry_msgs
)

install(TARGETS
  clicked_point_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
