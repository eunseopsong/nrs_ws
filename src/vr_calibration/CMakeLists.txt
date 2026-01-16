cmake_minimum_required(VERSION 3.8)
project(vr_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# =========================
# Include dirs (shared)
# =========================
include_directories(
  include
)

# =========================
# Executables
# =========================

# (1) Auto-capture node
add_executable(vr_calibration
  src/vr_calibration.cpp
)
ament_target_dependencies(vr_calibration
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
)
target_link_libraries(vr_calibration
  yaml-cpp
  Eigen3::Eigen
)
target_include_directories(vr_calibration PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# (2) Offline solver (separate main)
add_executable(vr_calibration_solver
  src/vr_calibration_solver.cpp
)
ament_target_dependencies(vr_calibration_solver
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
)
target_link_libraries(vr_calibration_solver
  yaml-cpp
  Eigen3::Eigen
)
target_include_directories(vr_calibration_solver PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# =========================
# Install
# =========================
install(TARGETS
  vr_calibration
  vr_calibration_solver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# 네가 쓰던 그대로 유지 (data/ 폴더를 share/${PROJECT_NAME}/yaml 로 설치)
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/yaml
)

ament_package()
