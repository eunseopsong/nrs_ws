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
find_package(Eigen3 REQUIRED)  # ✅ Eigen3 추가

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}       # ✅ Eigen include 경로 추가
)

# 노드 실행 파일
add_executable(inverse_kinematics_node src/inverse_kinematics_node.cpp)
ament_target_dependencies(inverse_kinematics_node
  rclcpp std_msgs sensor_msgs
)

add_executable(forward_kinematics_node src/forward_kinematics_node.cpp)
ament_target_dependencies(forward_kinematics_node
  rclcpp std_msgs
)

add_executable(txt_trajectory_executor src/txt_trajectory_executor.cpp)
ament_target_dependencies(txt_trajectory_executor rclcpp sensor_msgs Eigen3)
install(TARGETS txt_trajectory_executor DESTINATION lib/${PROJECT_NAME})


add_executable(file_reader_node src/file_reader_node.cpp)
ament_target_dependencies(file_reader_node rclcpp)
install(TARGETS file_reader_node DESTINATION lib/${PROJECT_NAME})



# 설치 설정
install(TARGETS
  inverse_kinematics_node
  forward_kinematics_node
  DESTINATION lib/${PROJECT_NAME}
)

# 패키지 export
ament_package()
