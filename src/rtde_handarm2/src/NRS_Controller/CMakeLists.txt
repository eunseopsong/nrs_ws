include_directories("./")
include_directories("./Controllers")
# include_directories("./FSM_States")

# include_directories("../../third-party/")
# include_directories(SYSTEM "../../third-party/qpOASES/include")
# include_directories("../../third-party/ParamHandler")

# include_directories(
#   ${GAZEBO_INCLUDE_DIRS}
#   ${qpOASES_INCLUDE_DIR}
#   ${PROJECT_SOURCE_DIR}
#   ${CMAKE_BINARY_DIR}
# )

# ------- ROS packages -------- #
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
find_package(nrs_forcecon2    REQUIRED)
find_package(nrs_msgmonitoring2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(ur_rtde REQUIRED PATHS "/home/nrsur10/ur_rtde/build/ur_rtde")



file(GLOB_RECURSE sources
"./*.cpp"
# "FSM_States/*.cpp"
"Controllers/FAAC/*.cpp"
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

add_executable(nrs_ctrl ${sources} ur10e_main.cpp)
target_include_directories(nrs_ctrl PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>) # — 헤더 검색 경로 추가
target_link_libraries(nrs_ctrl yaml-cpp Eigen3::Eigen)
ament_target_dependencies(nrs_ctrl ${COMMON_DEPENDENCIES})

# Link the necessary libraries to the executable
target_link_libraries(nrs_ctrl
#   ${GAZEBO_LIBRARIES}
  ${rclcpp_LIBRARIES}
#   ${qpOASES_LIBRARIES}
)

# Optionally install the executable
install(TARGETS nrs_ctrl
  DESTINATION lib/${PROJECT_NAME}
)

# add_subdirectory(Controllers/FAAC)
# add_subdirectory(Controllers/WBC_Ctrl)
# add_subdirectory(Controllers/VisionMPC)

# add_executable(aidin_ctrl ${sources} AiDIN_Controller.cpp main.cpp)
# target_link_libraries(mit_ctrl robot biomimetics)
# target_link_libraries(aidin_ctrl dynacore_param_handler qpOASES)
# target_link_libraries(mit_ctrl Goldfarb_Optimizer osqp)
# target_link_libraries(mit_ctrl WBC_Ctrl)
# target_link_libraries(mit_ctrl VisionMPC)
