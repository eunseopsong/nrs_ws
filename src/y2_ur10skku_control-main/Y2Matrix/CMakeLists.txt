cmake_minimum_required(VERSION 3.8)
project(Y2Matrix)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# include directory
include_directories(include)

# package libraries
add_library(${PROJECT_NAME} src/YMatrix.cpp src/RotationTransform.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} rclcpp)

# package execution
add_executable(basic_example src/Examples/Basic_Example.cpp)
ament_target_dependencies(basic_example rclcpp)
target_link_libraries(basic_example ${PROJECT_NAME})

add_executable(rotTrans_example src/Examples/RotTrans_Example.cpp)
ament_target_dependencies(rotTrans_example rclcpp)
target_link_libraries(rotTrans_example ${PROJECT_NAME})


# Install library target
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install execution target
install(TARGETS basic_example rotTrans_example
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include/
)

# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # Custom library
ament_export_dependencies(rclcpp) # ROS library



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
