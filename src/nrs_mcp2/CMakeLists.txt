cmake_minimum_required(VERSION 3.8)
project(nrs_mcp2)

cmake_policy(SET CMP0074 NEW)

# 컴파일러 설정 (C++17 필수)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# MoveIt2 및 최신 ROS 2 라이브러리는 C++17을 권장/요구합니다.
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
find_package(sensor_msgs REQUIRED)     # PointCloud2 사용을 위해 필요
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)       # get_tcp_pose_server에서 사용
find_package(Eigen3 REQUIRED)          # get_tcp_pose_server에서 사용
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# MoveIt 관련 의존성
find_package(moveit_ros_planning_interface REQUIRED)

# PCL (pcd_publisher에서 사용)
find_package(PCL REQUIRED)

# 커스텀 메시지/서비스 생성을 위한 패키지
find_package(rosidl_default_generators REQUIRED)

# ========================================
# 2. 인터페이스(서비스) 생성
# ========================================
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
add_executable(moveit_go_to_pose_server src/moveit_go_to_pose_server.cpp)
ament_target_dependencies(moveit_go_to_pose_server
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
  tf2_ros
)
# 같은 패키지 내의 서비스(GoToPose) 헤더 생성을 기다리기 위해 의존성 추가
rosidl_target_interfaces(moveit_go_to_pose_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# --- 3.2 Get TCP Pose Server ---
add_executable(get_tcp_pose_server src/get_tcp_pose_server.cpp)
ament_target_dependencies(get_tcp_pose_server
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen        # [필수] Eigen 변환 함수 사용
  Eigen3           # [필수] Eigen 라이브러리
)
# Eigen 헤더 경로 명시적 포함
target_include_directories(get_tcp_pose_server PUBLIC ${Eigen3_INCLUDE_DIRS})
# 서비스 헤더 의존성
rosidl_target_interfaces(get_tcp_pose_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# --- 3.3 PCD Publisher ---
add_executable(pcd_publisher src/pcd_publisher.cpp)
ament_target_dependencies(pcd_publisher
  rclcpp
  pcl_ros
  pcl_conversions
  std_msgs
  sensor_msgs
)
# PCL 헤더 경로 및 라이브러리 링크 (중요)
target_include_directories(pcd_publisher PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_publisher ${PCL_LIBRARIES})

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
# Central Supervisor도 GoToPose 서비스를 클라이언트로 사용한다면 헤더 생성을 기다려야 함
rosidl_target_interfaces(central_supervisor_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# ========================================
# 4. 설치 규칙 (Install Rules)
# ========================================

# 실행 파일 설치
install(TARGETS
  moveit_go_to_pose_server
  get_tcp_pose_server
  pcd_publisher
  clicked_points_logger
  central_supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Launch 파일 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 인터페이스 파일 원본 설치
install(FILES
  srv/GoToPose.srv
  srv/GetTcpPose.srv
  DESTINATION share/${PROJECT_NAME}/srv
)

# 설정 파일 등이 있다면 설치 (예: ur10_calibration.yaml)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/ur10_calibration.yaml")
  install(FILES ur10_calibration.yaml DESTINATION share/${PROJECT_NAME})
endif()

# 빌드 시스템 내보내기
ament_export_dependencies(rosidl_default_runtime)
ament_package()