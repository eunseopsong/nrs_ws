cmake_minimum_required(VERSION 3.5)
project(nrs_vision2)

# C++ 표준을 C++17로 설정 (필요 시 수정)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

# 실행 파일 생성 (src 폴더에 moveit_isaac_integration.cpp가 위치)
add_executable(moveit_isaac_integration src/moveit_isaac_integration.cpp)

# 타겟에 의존성 연결
ament_target_dependencies(moveit_isaac_integration
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# 실행 파일 설치 위치 지정
install(TARGETS
  moveit_isaac_integration
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
