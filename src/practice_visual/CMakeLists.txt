cmake_minimum_required(VERSION 3.8)
project(practice_visual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ⭐️ 추가 1: Eigen3 라이브러리 찾기 ⭐️
find_package(Eigen3 REQUIRED)

# ⭐️ 추가 2: 헤더 파일 경로에 Eigen3 추가 ⭐️
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# 실행 파일(노드) 만들기
add_executable(ifs_node
  src/main_ifs.cpp
  src/ifs_algorithm.cpp
  src/stl_loader.cpp
)

# 의존성 연결
ament_target_dependencies(ifs_node
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2_ros
)

# 런치, 설정, 모델 파일들 복사 규칙 (아까 하신 것)
install(DIRECTORY urdf meshes launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# 실행 파일 설치 규칙
install(TARGETS ifs_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()