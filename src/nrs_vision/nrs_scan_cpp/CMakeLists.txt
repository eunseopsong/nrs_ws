cmake_minimum_required(VERSION 3.8)
project(nrs_scan_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(y2_rob_motion_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters features search kdtree surface)
find_package(OpenCV REQUIRED)

add_definitions(${PCL_DEFINITIONS})
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(scan_capture_node src/scan_capture_node.cpp)
ament_target_dependencies(
  scan_capture_node
  rclcpp
  sensor_msgs
  y2_rob_motion_interfaces
  pcl_conversions
)
target_link_libraries(
  scan_capture_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(reconstruct_node src/reconstruct_node.cpp)
ament_target_dependencies(reconstruct_node rclcpp)
target_link_libraries(
  reconstruct_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS
  scan_capture_node
  reconstruct_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config isaac scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
