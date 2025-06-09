cmake_minimum_required(VERSION 3.8)
project(nrs_msgmonitoring2)

# C++17
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(std_msgs     REQUIRED)
find_package(yaml-cpp     REQUIRED)

# Include dirs
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
  rclcpp std_msgs yaml-cpp
)

# (Optional) Executable test
add_executable(test_monitor src/test_main.cpp)
target_link_libraries(test_monitor msg_monitoring)
ament_target_dependencies(test_monitor
  rclcpp std_msgs yaml-cpp
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install targets and export for find_package()
install(
  TARGETS
    msg_monitoring
    test_monitor
  EXPORT nrs_msgmonitoring2Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  EXPORT nrs_msgmonitoring2Targets
  FILE nrs_msgmonitoring2Targets.cmake
  NAMESPACE nrs_msgmonitoring2::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Export to ament
ament_export_libraries(msg_monitoring)
ament_export_dependencies(rclcpp std_msgs yaml-cpp)
ament_export_include_directories(include)

# This generates the ConfigVersion and makes the package findable
ament_package()
