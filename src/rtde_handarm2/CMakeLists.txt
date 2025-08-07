cmake_minimum_required(VERSION 3.8)
project(rtde_handarm2)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ──────────────── ROS & 외부 패키지 ──────────────── #
find_package(ament_cmake      REQUIRED)
find_package(rclcpp           REQUIRED)
find_package(std_msgs         REQUIRED)
find_package(std_srvs         REQUIRED)
find_package(geometry_msgs    REQUIRED)
find_package(sensor_msgs      REQUIRED)
find_package(tf2              REQUIRED)
find_package(tf2_ros          REQUIRED)
find_package(yaml-cpp         REQUIRED)
find_package(Eigen3           REQUIRED)
find_package(ament_index_cpp  REQUIRED)

find_package(nrs_forcecon2      REQUIRED)
find_package(nrs_msgmonitoring2 REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# ──────────────── 메시지 정의 ──────────────── #
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmMsg.msg"
  "msg/FTsensorMsg.msg"
  "msg/ViveFTMsg.msg"
  "msg/VrPosRtMsgRpy.msg"
  "msg/VrPosRtMsgQua.msg"
  DEPENDENCIES std_msgs
)

# ──────────────── include 디렉토리 설정 ──────────────── #
include_directories(
  include
  include/rtde_handarm2/Arm_guide
  include/rtde_handarm2/Armtraj
  include/rtde_handarm2/UR10
  include/rtde_handarm2/Yoon_filters
  include/rtde_handarm2/Yoon_FT_sensor
  include/rtde_handarm2/Yoon_path
  include/rtde_handarm2/Yoon_force_control
  include/rtde_handarm2/Yoon_communication
  include/rtde_handarm2/Yoon_data_loader
  include/rtde_handarm2/Third_party/Eigen3/eigen
  ${CMAKE_CURRENT_SOURCE_DIR}/yaml    # ✅ 여기에 추가
)

# ──────────────── 소스 파일 수집 ──────────────── #
file(GLOB_RECURSE MODULE_SRCS
  "include/rtde_handarm2/*/*.cpp"
)

file(GLOB_RECURSE SOURCES
  "src/*.cpp"
)

# 제외할 노드 소스 파일들 정의
set(EXCLUDE_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/node_ur10e_main.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/Yoon_UR10e_cmd.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/node_Hbutton.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/node_path_receiver.cpp"
)

list(REMOVE_ITEM SOURCES ${EXCLUDE_SOURCES})

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

# ──────────────── 노드별 실행파일 추가 ──────────────── #
add_executable(nrs_ctrl src/node_ur10e_main.cpp ${SOURCES} ${MODULE_SRCS})
target_include_directories(nrs_ctrl PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(nrs_ctrl yaml-cpp Eigen3::Eigen ament_index_cpp::ament_index_cpp)
ament_target_dependencies(nrs_ctrl ${COMMON_DEPENDENCIES})
install(TARGETS nrs_ctrl DESTINATION lib/${PROJECT_NAME})

add_executable(ur10e_mode_cmd src/Yoon_UR10e_cmd.cpp ${SOURCES} ${MODULE_SRCS})
target_include_directories(ur10e_mode_cmd PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(ur10e_mode_cmd yaml-cpp Eigen3::Eigen ament_index_cpp::ament_index_cpp)
ament_target_dependencies(ur10e_mode_cmd ${COMMON_DEPENDENCIES})
install(TARGETS ur10e_mode_cmd DESTINATION lib/${PROJECT_NAME})

add_executable(hbutton src/node_Hbutton.cpp ${SOURCES} ${MODULE_SRCS})
target_include_directories(hbutton PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(hbutton yaml-cpp Eigen3::Eigen ament_index_cpp::ament_index_cpp)
ament_target_dependencies(hbutton ${COMMON_DEPENDENCIES})
install(TARGETS hbutton DESTINATION lib/${PROJECT_NAME})

add_executable(path_receiver src/node_path_receiver.cpp)
ament_target_dependencies(path_receiver rclcpp std_msgs)
install(TARGETS path_receiver DESTINATION lib/${PROJECT_NAME})

# ──────────────── 설치 파일들 ──────────────── #
install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

# ──────────────── ament 패키지 설정 ──────────────── #
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
ament_export_include_directories(include)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()
