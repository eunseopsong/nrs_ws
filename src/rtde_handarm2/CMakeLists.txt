cmake_minimum_required(VERSION 3.8)
project(rtde_handarm2)

# C++ 표준
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 메시지 생성
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armmsg.msg"
  "msg/ftsensorMsg.msg"
  "msg/VRposRtMsg_RPY.msg"
  "msg/VRposRtMsg_Qua.msg"
  DEPENDENCIES std_msgs
)

# 의존성 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ur_rtde REQUIRED PATHS "/home/nrsur10/ur_rtde/build/ur_rtde")

# include 디렉토리
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

# 사용자 정의 라이브러리 디렉토리 등록
set(Arm_guide_dir ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm/Arm_guide)
set(Armtraj_dir ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm/Armtraj)
set(UR10_dir ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm/UR10)
set(Yoon_filters_dir ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm/Yoon_filters)
set(Yoon_FT_sensor_dir ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm/Yoon_FT_sensor)
set(Yoon_path_dir ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm/Yoon_path)
set(Yoon_force_control_dir ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm/Yoon_force_control)
set(Yoon_communication_dir ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm/Yoon_communication)
set(Yoon_data_loader_dir ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm/Yoon_data_loader)

# add_subdirectory 대신 각 라이브러리를 개별적으로 src로 이관 후 빌드 예정

# 실행 노드 예시 (이후 각 .cpp 추가하면서 완성)
add_executable(rtde_test src/rtde_test.cpp)
ament_target_dependencies(rtde_test
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  yaml-cpp
  ur_rtde
  Eigen3
)
# 필요시 타 라이브러리 링크
# target_link_libraries(rtde_test ${CUSTOM_LIBS})

# 설치 설정
install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  rtde_test
  DESTINATION lib/${PROJECT_NAME}
)

# 메시지 설치
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(rtde_test ${cpp_typesupport_target})

# 패키지 선언
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  yaml-cpp
  ur_rtde
  Eigen3
)
ament_package()
