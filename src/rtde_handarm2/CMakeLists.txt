cmake_minimum_required(VERSION 3.8)
project(rtde_handarm2)

# Include directories
include_directories("src/")
include_directories("include/")
include_directories("include/rtde_handarm2")
include_directories("yaml")  # ← yaml 경로 추가!

# ROS2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nrs_forcecon2 REQUIRED)
find_package(nrs_msgmonitoring2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Export dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  yaml-cpp
  Eigen3
  rosidl_default_runtime
)
ament_export_include_directories(include yaml)

# ---------- 외부 C++ 소스들 모음 ----------
set(Yoon_LIB
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/Armtraj/Armtraj.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/Arm_guide/ArmGuide.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/UR10/Kinematics.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/Yoon_force_control/Yoon_force_control.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/Yoon_path/Yoon_path.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/Yoon_filters/Yoon_filters.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/Yoon_FT_sensor/Yoon_FT_sensor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/Yoon_data_loader/Text_loader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/Yoon_communication/Yoon_communi.cpp
)

# 공통 의존성
set(COMMON_DEPENDENCIES
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  nrs_forcecon2
  nrs_msgmonitoring2
)

# 전체 소스 중 main 없는 파일만 모음
file(GLOB_RECURSE sources
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# main 포함된 파일 목록
set(MAIN_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/node_ur10e_main.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/node_Hbutton.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/Yoon_UR10e_cmd.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/node_path_receiver.cpp"
)

# 제외
list(REMOVE_ITEM sources ${MAIN_FILES})

# ──────────────── 실행파일 생성 ──────────────── #

# [1] nrs_ctrl (Main node)
add_executable(nrs_ctrl
  node_ur10e_main.cpp
  ${sources}
  ${Yoon_LIB}
)
target_include_directories(nrs_ctrl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/yaml>
)
target_link_libraries(nrs_ctrl yaml-cpp Eigen3::Eigen)
ament_target_dependencies(nrs_ctrl ${COMMON_DEPENDENCIES})
install(TARGETS nrs_ctrl DESTINATION lib/${PROJECT_NAME})

# [2] ur10e_mode_cmd
add_executable(ur10e_mode_cmd
  Yoon_UR10e_cmd.cpp
  ${sources}
  ${Yoon_LIB}
)
target_include_directories(ur10e_mode_cmd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/yaml>
)
target_link_libraries(ur10e_mode_cmd yaml-cpp Eigen3::Eigen)
ament_target_dependencies(ur10e_mode_cmd ${COMMON_DEPENDENCIES})
install(TARGETS ur10e_mode_cmd DESTINATION lib/${PROJECT_NAME})

# [3] hbutton
add_executable(hbutton
  node_Hbutton.cpp
  ${sources}
  ${Yoon_LIB}
)
target_include_directories(hbutton PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/yaml>
)
target_link_libraries(hbutton yaml-cpp Eigen3::Eigen)
ament_target_dependencies(hbutton ${COMMON_DEPENDENCIES})
install(TARGETS hbutton DESTINATION lib/${PROJECT_NAME})

# [4] path_receiver (간단한 노드)
add_executable(path_receiver
  node_path_receiver.cpp
)
ament_target_dependencies(path_receiver rclcpp std_msgs)
install(TARGETS path_receiver DESTINATION lib/${PROJECT_NAME})

# 메시지 타입 지원 (옵션)
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 설치 설정 (headers, launch, data 등)
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

install(DIRECTORY yaml/
  DESTINATION share/${PROJECT_NAME}/yaml
)

# ament 설정
ament_package()
