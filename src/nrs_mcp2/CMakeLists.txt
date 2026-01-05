cmake_minimum_required(VERSION 3.8)
project(nrs_mcp2)

cmake_policy(SET CMP0074 NEW)

# 컴파일러 설정 (C++17 필수)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# MoveIt2 등 최신 라이브러리는 C++17을 요구합니다.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ========================================
# 1. 패키지 의존성 찾기 (Find Packages)
# ========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# MoveIt 관련 의존성 (ROS 2 버전에 따라 패키지명이 다를 수 있으나 보통 이것을 사용)
find_package(moveit_ros_planning_interface REQUIRED)

# 커스텀 메시지/서비스 생성을 위한 패키지
find_package(rosidl_default_generators REQUIRED)

# PCL 및 Eigen (필요 시 명시적 포함)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# ========================================
# 2. 인터페이스(메시지/서비스) 생성
# ========================================
# ROS 2에서는 add_service_files 대신 rosidl_generate_interfaces를 사용합니다.
# .srv 파일들은 srv/ 폴더 안에 있어야 합니다.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToPose.srv"
  "srv/GetTcpPose.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# ========================================
# 3. 실행 파일 빌드 (Executables)
# ========================================

# --- 3.1 MoveIt Go To Pose Server ---
# add_executable(moveit_go_to_pose_server src/moveit_go_to_pose_server.cpp)
# ament_target_dependencies(moveit_go_to_pose_server
#   rclcpp
#   geometry_msgs
#   moveit_ros_planning_interface
#   tf2_geometry_msgs
#   tf2_ros
# )
# # 같은 패키지 내의 서비스(GoToPose)를 사용하므로 의존성 추가
# rosidl_target_interfaces(moveit_go_to_pose_server
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )

# --- 3.2 Get TCP Pose Server ---
# add_executable(get_tcp_pose_server src/get_tcp_pose_server.cpp)
# ament_target_dependencies(get_tcp_pose_server
#   rclcpp
#   geometry_msgs
#   tf2_ros
#   tf2_geometry_msgs
# )
# rosidl_target_interfaces(get_tcp_pose_server
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )

# --- 3.3 PCD Publisher ---
# add_executable(pcd_publisher src/pcd_publisher.cpp)
# ament_target_dependencies(pcd_publisher
#   rclcpp
#   pcl_ros
#   pcl_conversions
#   std_msgs
#   sensor_msgs
# )
# PCL 헤더 경로 포함
# target_include_directories(pcd_publisher PUBLIC ${PCL_INCLUDE_DIRS})
# target_link_libraries(pcd_publisher ${PCL_LIBRARIES})

# --- 3.4 Clicked Points Logger ---
add_executable(clicked_points_logger src/clicked_points_logger.cpp)
ament_target_dependencies(clicked_points_logger
  rclcpp
  visualization_msgs
  geometry_msgs
)

# --- 3.5 Central Supervisor (앞서 변환한 파일) ---
add_executable(central_supervisor_node src/central_supervisor.cpp)
ament_target_dependencies(central_supervisor_node
  rclcpp
  std_srvs
)
# Central Supervisor는 자체 정의 서비스(Empty)가 아니라 std_srvs를 쓰므로
# 추가적인 rosidl_target_interfaces 설정은 필요 없습니다.

# ========================================
# 4. 설치 규칙 (Install Rules)
# ========================================

# 실행 파일 설치
install(TARGETS
  # moveit_go_to_pose_server
  # get_tcp_pose_server
  # pcd_publisher
  clicked_points_logger
  central_supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Launch 파일 설치 (Python launch 파일 사용 권장)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 인터페이스 파일 원본 설치 (선택 사항이지만 권장)
install(FILES
  srv/GoToPose.srv
  srv/GetTcpPose.srv
  DESTINATION share/${PROJECT_NAME}/srv
)

# 빌드 시스템 내보내기
ament_export_dependencies(rosidl_default_runtime)
ament_package()