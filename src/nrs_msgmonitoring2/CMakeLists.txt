cmake_minimum_required(VERSION 3.8)
project(nrs_msgmonitoring2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${yaml-cpp_INCLUDE_DIRS}
)

# Library
add_library(msg_monitoring
  src/msg_monitoring.cpp
)

target_include_directories(msg_monitoring PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(msg_monitoring
  rclcpp
  std_msgs
  yaml-cpp
)

# (Optional) Executable test
add_executable(test_monitor src/test_main.cpp)
target_link_libraries(test_monitor msg_monitoring)
ament_target_dependencies(test_monitor rclcpp std_msgs yaml-cpp)

# Install
install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  msg_monitoring
  test_monitor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
