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

find_package(Eigen3 REQUIRED)        # ✅ 여기 대문자!
find_package(yaml-cpp REQUIRED)

# =========================
# Executable
# =========================
add_executable(vr_calibration
  src/vr_calibration.cpp
  src/vr_calibration_solver.cpp
)

ament_target_dependencies(vr_calibration
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
)

target_link_libraries(vr_calibration
  yaml-cpp
  Eigen3::Eigen           # ✅ 권장 (헤더-only지만 타겟으로 연결)
)

target_include_directories(vr_calibration PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# =========================
# Install
# =========================
install(TARGETS vr_calibration
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/yaml
)

ament_package()
