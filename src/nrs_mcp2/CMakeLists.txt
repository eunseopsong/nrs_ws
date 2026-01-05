cmake_minimum_required(VERSION 3.8)
project(nrs_mcp2)

cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ========================================
# 1. 패키지 의존성 찾기
# ========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)       # 필수
find_package(Eigen3 REQUIRED)          # 필수
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# MoveIt 관련 (나중을 위해 유지)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(PCL REQUIRED)

# ========================================
# 2. 인터페이스(서비스) 생성
# ========================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToPose.srv"
  "srv/GetTcpPose.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# ========================================
# 3. 실행 파일 빌드
# ========================================

# --- 3.1 MoveIt Go To Pose Server (아직 주석) ---
# add_executable(moveit_go_to_pose_server src/moveit_go_to_pose_server.cpp)
# ament_target_dependencies(moveit_go_to_pose_server
#   rclcpp
#   geometry_msgs
#   moveit_ros_planning_interface
#   tf2_geometry_msgs
#   tf2_ros
# )
# rosidl_target_interfaces(moveit_go_to_pose_server
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )

# --- 3.2 Get TCP Pose Server (수정됨!) ---
add_executable(get_tcp_pose_server src/get_tcp_pose_server.cpp)
ament_target_dependencies(get_tcp_pose_server
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen        # <--- [중요] 추가됨
  Eigen3           # <--- [중요] 추가됨
)
# Eigen 헤더 경로를 확실하게 포함
target_include_directories(get_tcp_pose_server PUBLIC ${Eigen3_INCLUDE_DIRS})

rosidl_target_interfaces(get_tcp_pose_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# --- 3.3 PCD Publisher (아직 주석) ---
# add_executable(pcd_publisher src/pcd_publisher.cpp)
# ...

# --- 3.4 Clicked Points Logger ---
add_executable(clicked_points_logger src/clicked_points_logger.cpp)
ament_target_dependencies(clicked_points_logger
  rclcpp
  visualization_msgs
  geometry_msgs
)

# --- 3.5 Central Supervisor ---
add_executable(central_supervisor_node src/central_supervisor.cpp)
ament_target_dependencies(central_supervisor_node
  rclcpp
  std_srvs
)

# ========================================
# 4. 설치 규칙
# ========================================
install(TARGETS
  # moveit_go_to_pose_server
  get_tcp_pose_server
  # pcd_publisher
  clicked_points_logger
  central_supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  srv/GoToPose.srv
  srv/GetTcpPose.srv
  DESTINATION share/${PROJECT_NAME}/srv
)

# 파일이 실제 존재할 때만 설치 (없으면 에러 방지)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/ur10_calibration.yaml")
  install(FILES ur10_calibration.yaml DESTINATION share/${PROJECT_NAME})
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()