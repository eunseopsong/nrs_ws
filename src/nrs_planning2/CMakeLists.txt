cmake_minimum_required(VERSION 3.5)
project(nrs_planning2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include header files (if any)
include_directories(
  include
)

# Create the executable
add_executable(interpolation_node
  src/interpolation_node.cpp
)

# Link dependencies
ament_target_dependencies(
  interpolation_node
  rclcpp
  std_msgs
)

# If you have headers under include/nrs_planning2, install them:
# install(DIRECTORY include/
#   DESTINATION include
# )

# Install the executable to lib/<project_name>
install(TARGETS interpolation_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
