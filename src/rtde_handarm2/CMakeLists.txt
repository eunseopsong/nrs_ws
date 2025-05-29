cmake_minimum_required(VERSION 3.8)
project(rtde_handarm2)

# C++ 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 필수 의존성
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# 실시간성을 보장하기 위한 package (ros2에서는 필요없어짐)
## find_package(ur_rtde REQUIRED PATHS "/home/nrsur10/ur_rtde/build/ur_rtde")
# find_package(ur_rtde REQUIRED PATHS "/home/nrsur10/ur_rtde/build/ur_rtde")

# 메시지 정의
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmMsg.msg"
  "msg/FtsensorMsg.msg"
  "msg/VrPosRtMsgRpy.msg"
  "msg/VrPosRtMsgQua.msg"
  DEPENDENCIES std_msgs
)

# include 디렉토리 설정
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

# 실행 노드 예시
# add_executable(rtde_test src/rtde_test.cpp)
# ament_target_dependencies(rtde_test
#   rclcpp
#   std_msgs
#   geometry_msgs
#   sensor_msgs
#   yaml-cpp
#   Eigen3
# )

install(DIRECTORY include/
  DESTINATION include/
)

# ur_rtde 라이브러리 명시적으로 추가
# target_link_libraries(rtde_test
#   ${cpp_typesupport_target}
#   ${YAML_CPP_LIBRARIES}
# )

# 패키지 export
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  yaml-cpp
  Eigen3
)
ament_package()
