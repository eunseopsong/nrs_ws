cmake_minimum_required(VERSION 3.8)
project(contact_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(feasibility_test_node src/feasibility_test_node.cpp)
target_compile_features(feasibility_test_node PUBLIC cxx_std_17)
ament_target_dependencies(feasibility_test_node
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(feasibility_test_node2 src/feasibility_test_node2.cpp)
target_compile_features(feasibility_test_node2 PUBLIC cxx_std_17)
ament_target_dependencies(feasibility_test_node2
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  feasibility_test_node
  feasibility_test_node2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
