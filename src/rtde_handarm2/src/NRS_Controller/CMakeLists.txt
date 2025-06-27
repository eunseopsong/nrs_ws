include_directories("./")
include_directories("./Controllers")
include_directories("../../include/")
include_directories("../../include/rtde_handarm2")

# ROS dependencies
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

# 외부 모듈 소스 직접 추가
# set(Yoon_LIB ArmGuide Kinematics Armtraj Yoon_filters Yoon_FT_sensor Yoon_path Yoon_force_control Text_loader Yoon_communication)
set(Yoon_LIB
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include/rtde_handarm2/Armtraj/Armtraj.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include/rtde_handarm2/Arm_guide/ArmGuide.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include/rtde_handarm2/UR10/Kinematics.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include/rtde_handarm2/Yoon_force_control/Yoon_force_control.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include/rtde_handarm2/Yoon_path/Yoon_path.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include/rtde_handarm2/Yoon_filters/Yoon_filters.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include/rtde_handarm2/Yoon_FT_sensor/Yoon_FT_sensor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include/rtde_handarm2/Yoon_data_loader/Text_loader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include/rtde_handarm2/Yoon_communication/Yoon_communi.cpp
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


file(GLOB SUBSCRIBER_SRCS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/Yoon_UR10e_main_subs/*.cpp"
)

# 1. 모든 cpp 파일 GLOB (여기엔 node_*.cpp도 포함됨)
file(GLOB_RECURSE sources
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Controllers/FAAC/*.cpp"
)

# 2. 제외할 main 파일 명시
set(EXCLUDE_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/node_ur10e_main.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/node_Hbutton.cpp"
)

# 3. 리스트에서 제외
list(REMOVE_ITEM sources ${EXCLUDE_SOURCES})


# ------- ur10e_main node dependencies -------- #
add_executable(nrs_ctrl node_ur10e_main.cpp ${sources} ${Yoon_LIB} ${SUBSCRIBER_SRCS})
target_include_directories(nrs_ctrl PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(nrs_ctrl yaml-cpp Eigen3::Eigen)
ament_target_dependencies(nrs_ctrl ${COMMON_DEPENDENCIES})
install(TARGETS nrs_ctrl DESTINATION lib/${PROJECT_NAME})

# -------- Hbutton node dependencies --------- #
add_executable(hbutton node_Hbutton.cpp ${sources} ${Yoon_LIB} ${SUBSCRIBER_SRCS})
target_include_directories(hbutton PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(hbutton yaml-cpp Eigen3::Eigen)
ament_target_dependencies(hbutton ${COMMON_DEPENDENCIES})
install(TARGETS hbutton DESTINATION lib/${PROJECT_NAME})


rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
