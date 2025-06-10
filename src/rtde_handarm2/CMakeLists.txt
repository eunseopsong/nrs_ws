cmake_minimum_required(VERSION 3.8)
project(rtde_handarm2)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------- ROS packages -------- #

find_package(ament_cmake      REQUIRED)
find_package(rclcpp           REQUIRED)
find_package(std_msgs         REQUIRED)
find_package(geometry_msgs    REQUIRED)
find_package(sensor_msgs      REQUIRED)
find_package(tf2              REQUIRED)
find_package(tf2_ros          REQUIRED)
find_package(yaml-cpp         REQUIRED)      # yaml-cpp (non-namespaced)
find_package(Eigen3           REQUIRED)
find_package(nrs_forcecon2    REQUIRED)      # 이제 nrs_3step_faac 만 export 됩니다
find_package(nrs_msgmonitoring2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(ur_rtde REQUIRED PATHS "/home/nrsur10/ur_rtde/build/ur_rtde")

# -------------------------------#

# ------- RTDE packages -------- #
# set(THIS_PACKAGE_INCLUDE_DIRS doc/interactivity/include)  # Not used directly in ROS 2
# find_package(ur_rtde REQUIRED) # assumes installed system-wide or in CMAKE_PREFIX_PATH
# --------------------------------#

# ------- Eigen packages -------- #
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
  message(STATUS "Eigen3 Found! - ${Eigen3_DIR}")
endif()
# -------------------------------#

# ------- Yaml packages -------- #
if(yaml-cpp_FOUND)
  message(STATUS "yaml-cpp Found! - ${yaml-cpp_DIR}")
endif()
# -------------------------------#

# ------- For ROS msg usage -------- #
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmMsg.msg"
  "msg/FTsensorMsg.msg"
  "msg/ViveFTMsg.msg"
  "msg/VrPosRtMsgRpy.msg"
  "msg/VrPosRtMsgQua.msg"
  DEPENDENCIES std_msgs
)
# -------------------------------#

# ------- Add to package exports -------- #
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  yaml-cpp
  # ur_rtde
  Eigen3
  rosidl_default_runtime
)
ament_export_include_directories(include)
# -------------------------------#

# include directories for all custom modules
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
)

include_directories(
  include
  include/rtde_handarm2/Arm_guide
  include/rtde_handarm2/Armtraj
  include/rtde_handarm2/UR10
  include/rtde_handarm2/Yoon_communication
  include/rtde_handarm2/Yoon_data_loader
  include/rtde_handarm2/Yoon_filters
  include/rtde_handarm2/Yoon_force_control
  include/rtde_handarm2/Yoon_FT_sensor
  include/rtde_handarm2/Yoon_path
  include/rtde_handarm2/Third_party/Eigen3/eigen
)

# For folder directories
include_directories(
  src/Yoon_UR10e_main_subs
  ${CMAKE_CURRENT_SOURCE_DIR}/NRS_yaml
)

# — Yoon_LIB 그룹
set(Yoon_LIB
  ArmGuide
  Kinematics
  Armtraj
  Yoon_filters
  Yoon_FT_sensor
  Yoon_path
  Yoon_force_control
  Text_loader
  Yoon_communication
)

# Executables and their dependencies

### add_executable(rtde_test src/rtde_test.cpp)
### target_link_libraries(rtde_test ur_rtde::rtde ${Yoon_LIB} yaml-cpp::yaml-cpp Eigen3::Eigen)
### ament_target_dependencies(rtde_test rclcpp std_msgs geometry_msgs sensor_msgs tf2 tf2_ros)

#######
file(GLOB_RECURSE MODULE_SRCS
  "${CMAKE_CURRENT_SOURCE_DIR}/include/rtde_handarm2/*/*.cpp"
)
# 콜백 구현 파일들을 포함하도록 Subscriber 소스 glob 추가
file(GLOB SUBSCRIBER_SRCS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/Yoon_UR10e_main_subs/*.cpp"
)
# 공통 dependencies 변수 정의
set(COMMON_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  nrs_forcecon2
  nrs_msgmonitoring2
)

######################################
#### Yoon_UR10e_main Dependencies ####
######################################
add_executable(Yoon_UR10e_main src/Yoon_UR10e_main.cpp src/globals.cpp ${MODULE_SRCS} ${SUBSCRIBER_SRCS})
target_include_directories(Yoon_UR10e_main PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>) # — 헤더 검색 경로 추가
target_link_libraries(Yoon_UR10e_main yaml-cpp Eigen3::Eigen) ## target_link_libraries(Yoon_UR10e_main ${Yoon_LIB} yaml-cpp::yaml-cpp Eigen3::Eigen)
ament_target_dependencies(Yoon_UR10e_main ${COMMON_DEPENDENCIES})


# add_executable(Yoon_FT_acquisit src/Yoon_FT_acquisit.cpp)
# target_link_libraries(Yoon_FT_acquisit ${Yoon_LIB} yaml-cpp::yaml-cpp)
# ament_target_dependencies(Yoon_FT_acquisit rclcpp std_msgs)

# add_executable(Yoon_UR10e_cmd src/Yoon_UR10e_cmd.cpp)
# target_link_libraries(Yoon_UR10e_cmd ${Yoon_LIB} yaml-cpp::yaml-cpp Eigen3::Eigen)
# ament_target_dependencies(Yoon_UR10e_cmd rclcpp std_msgs)

# add_executable(Yoon_Hbutton_cmd src/Yoon_Hbutton_cmd.cpp)
# target_link_libraries(Yoon_Hbutton_cmd ${Yoon_LIB} yaml-cpp::yaml-cpp Eigen3::Eigen)
# ament_target_dependencies(Yoon_Hbutton_cmd rclcpp std_msgs)


######################################
#### NRS_Hbutton_cmd Dependencies ####
######################################
# add_executable(NRS_Hbutton_cmd src/NRS_Hbutton_cmd.cpp src/globals.cpp ${MODULE_SRCS} ${SUBSCRIBER_SRCS}) ## add_executable(NRS_Hbutton_cmd src/NRS_Hbutton_cmd.cpp)
# target_link_libraries(NRS_Hbutton_cmd ${Yoon_LIB} yaml-cpp Eigen3::Eigen)                                 ## target_link_libraries(NRS_Hbutton_cmd ${Yoon_LIB} yaml-cpp::yaml-cpp Eigen3::Eigen)
# ament_target_dependencies(NRS_Hbutton_cmd ${COMMON_DEPENDENCIES})                                         ## ament_target_dependencies(NRS_Hbutton_cmd rclcpp std_msgs)


# add_executable(VR_Hbutton_cmd src/VR_Hbutton_cmd.cpp)
# target_link_libraries(VR_Hbutton_cmd ${Yoon_LIB} yaml-cpp::yaml-cpp Eigen3::Eigen)
# ament_target_dependencies(VR_Hbutton_cmd rclcpp std_msgs)

# add_executable(euichan_path_receiver src/euichan_path_receiver.cpp)
# target_link_libraries(euichan_path_receiver)
# ament_target_dependencies(euichan_path_receiver rclcpp std_msgs)

# Install targets
install(TARGETS
  # rtde_test
  Yoon_UR10e_main
  # Yoon_FT_acquisit
  # Yoon_UR10e_cmd
  # Yoon_Hbutton_cmd
  # NRS_Hbutton_cmd
  # VR_Hbutton_cmd
  # euichan_path_receiver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Interface and package setup
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(rtde_test ${cpp_typesupport_target})
ament_package()
