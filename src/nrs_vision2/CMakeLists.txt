cmake_minimum_required(VERSION 3.5)
project(nrs_vision2)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 예시 실행 파일(토커, 리스너)
# add_executable(talker src/talker.cpp)
# ament_target_dependencies(talker rclcpp std_msgs)

# add_executable(listener src/listener.cpp)
# ament_target_dependencies(listener rclcpp std_msgs)

# 빌드된 실행 파일 설치
install(
  TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

# ★ 핵심: launch 폴더 전체를 설치
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
