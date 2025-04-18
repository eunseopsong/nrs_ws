cmake_minimum_required(VERSION 3.8)
project(nrs_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# 노드 실행 파일
add_executable(inverse_kinematics_node src/inverse_kinematics_node.cpp)
ament_target_dependencies(inverse_kinematics_node rclcpp std_msgs sensor_msgs)

# 설치 설정
install(TARGETS
  inverse_kinematics_node
  DESTINATION lib/${PROJECT_NAME}
)

# 패키지 export
ament_package()
