cmake_minimum_required(VERSION 3.8)
project(Y2Trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Y2Matrix REQUIRED)

# === Create libraries ===

# PositionInterpolation library
add_library(PositionInterpolation src/PositionInterpolation.cpp)
target_include_directories(PositionInterpolation PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(PositionInterpolation rclcpp Y2Matrix)

# QuaternionInterpolator library
add_library(QuaternionInterpolator src/QuaternionInterpolator.cpp)
target_include_directories(QuaternionInterpolator PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(QuaternionInterpolator rclcpp Y2Matrix)

# AccProfiler library
add_library(AccProfiler src/AccProfiler.cpp)
target_include_directories(AccProfiler PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(AccProfiler rclcpp Y2Matrix)

# MotionBlender6D library
add_library(MotionBlender6D src/MotionBlender6D.cpp)
target_include_directories(MotionBlender6D PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_link_libraries(MotionBlender6D 
PositionInterpolation 
QuaternionInterpolator 
AccProfiler)
ament_target_dependencies(MotionBlender6D rclcpp Y2Matrix)

# MotionBlender9D library
add_library(MotionBlender9D src/MotionBlender9D.cpp)
target_include_directories(MotionBlender9D PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_link_libraries(MotionBlender9D 
PositionInterpolation 
QuaternionInterpolator 
AccProfiler)
ament_target_dependencies(MotionBlender9D rclcpp Y2Matrix)

# === Create executables ===

# MotionBlender6D_example
add_executable(MotionBlender6D_example src/examples/MotionBlender6D_example.cpp)
target_link_libraries(MotionBlender6D_example MotionBlender6D)
ament_target_dependencies(MotionBlender6D_example rclcpp Y2Matrix)

# MotionBlender9D_example
add_executable(MotionBlender9D_example src/examples/MotionBlender9D_example.cpp)
target_link_libraries(MotionBlender9D_example MotionBlender9D)
ament_target_dependencies(MotionBlender9D_example rclcpp Y2Matrix)

# === Create install ===

# install libraries
install(TARGETS 
  PositionInterpolation QuaternionInterpolator AccProfiler
  MotionBlender6D MotionBlender9D
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# install executables
install(TARGETS MotionBlender6D_example MotionBlender9D_example
  DESTINATION lib/${PROJECT_NAME}
)

# install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Export ament targets & dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Y2Matrix)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
