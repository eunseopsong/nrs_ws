cmake_minimum_required(VERSION 3.8)
project(rtde_handarm2)

# C++ 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories(
  include
  include/rtde_handarm2
  include/rtde_handarm2/UR10
  include/rtde_handarm2/Yoon_data_loader
  include/rtde_handarm2/Yoon_filters
  include/rtde_handarm2/Yoon_path
  include/rtde_handarm2/Yoon_communication
  include/rtde_handarm2/Yoon_force_control
  include/rtde_handarm2/Yoon_FT_sensor
  include/rtde_handarm2/Armtraj
  include/rtde_handarm2/Arm_guide
  include/rtde_handarm2/ForceControl
  src
  yaml
)

# 외부 모듈 소스 직접 추가
set(Yoon_LIB
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/Armtraj/Armtraj.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/Arm_guide/ArmGuide.cpp

  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/UR10/Kinematics.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/UR10/ur10e_forward.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/UR10/ur10e_inverse.cpp

  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/ForceControl/admittance_control.cpp

  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/Yoon_force_control/Yoon_force_control.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/Yoon_path/Yoon_path.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/Yoon_filters/Yoon_filters.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/Yoon_FT_sensor/Yoon_FT_sensor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/Yoon_data_loader/Text_loader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/Yoon_communication/Yoon_communi.cpp
)

# 의존 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nrs_forcecon2 REQUIRED)
find_package(nrs_msgmonitoring2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C CXX)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

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

# 사용자 정의 메시지
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmMsg.msg"
  "msg/FTsensorMsg.msg"
  "msg/ViveFTMsg.msg"
  "msg/VrPosRtMsgRpy.msg"
  "msg/VrPosRtMsgQua.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

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

# 1. 모든 cpp 파일 GLOB
file(GLOB_RECURSE sources
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# 2. 제외할 main 파일 명시
set(EXCLUDE_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/node_ur10e_main.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/node_hbutton.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/Yoon_UR10e_cmd.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/node_path_receiver.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/node_joint_recorder.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/ur10e_act_data_gen.cpp"
)

# 3. 리스트에서 제외
list(REMOVE_ITEM sources ${EXCLUDE_SOURCES})

# 🔴 func_ur10e_main.cpp를 확실히 포함시키기
#   (반드시 rtde_handarm2/src/func_ur10e_main.cpp 경로에 있어야 함)
list(APPEND sources "${CMAKE_CURRENT_SOURCE_DIR}/src/func_ur10e_main.cpp")
list(APPEND sources "${CMAKE_CURRENT_SOURCE_DIR}/src/callback_ur10e_main.cpp")
list(REMOVE_DUPLICATES sources)

# ------- ur10e_main node -------- #
add_executable(nrs_ctrl src/node_ur10e_main.cpp ${sources} ${Yoon_LIB})
target_include_directories(nrs_ctrl PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(nrs_ctrl yaml-cpp Eigen3::Eigen)
ament_target_dependencies(nrs_ctrl ${COMMON_DEPENDENCIES})
install(TARGETS nrs_ctrl DESTINATION lib/${PROJECT_NAME})

# ------- ur10e_mode_cmd node -------- #
add_executable(ur10e_mode_cmd src/Yoon_UR10e_cmd.cpp ${sources} ${Yoon_LIB})
target_include_directories(ur10e_mode_cmd PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(ur10e_mode_cmd yaml-cpp Eigen3::Eigen)
ament_target_dependencies(ur10e_mode_cmd ${COMMON_DEPENDENCIES})
install(TARGETS ur10e_mode_cmd DESTINATION lib/${PROJECT_NAME})

# -------- Hbutton node --------- #
add_executable(hbutton src/node_hbutton.cpp ${sources} ${Yoon_LIB})
target_include_directories(hbutton PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(hbutton yaml-cpp Eigen3::Eigen)
ament_target_dependencies(hbutton ${COMMON_DEPENDENCIES})
install(TARGETS hbutton DESTINATION lib/${PROJECT_NAME})

# -------- path_receiver node --------- #
add_executable(path_receiver src/node_path_receiver.cpp)
ament_target_dependencies(path_receiver rclcpp std_msgs)
install(TARGETS path_receiver DESTINATION lib/${PROJECT_NAME})

# -------- joint_recorder node --------- #
add_executable(joint_recorder src/node_joint_recorder.cpp)
target_include_directories(joint_recorder PUBLIC ${HDF5_INCLUDE_DIRS})
target_link_libraries(joint_recorder ${HDF5_LIBRARIES})
ament_target_dependencies(joint_recorder rclcpp sensor_msgs)
install(TARGETS joint_recorder DESTINATION lib/${PROJECT_NAME})

# -------- act data gen node --------- #
add_executable(ur10e_act_data_gen src/ur10e_act_data_gen.cpp)
target_include_directories(ur10e_act_data_gen PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${HDF5_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(ur10e_act_data_gen
  ${HDF5_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(ur10e_act_data_gen
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)
install(TARGETS ur10e_act_data_gen DESTINATION lib/${PROJECT_NAME})

# install 설정
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY yaml/ DESTINATION share/${PROJECT_NAME}/yaml)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY data/ DESTINATION share/${PROJECT_NAME}/data)

ament_package()
