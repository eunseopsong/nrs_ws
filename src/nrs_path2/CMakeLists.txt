cmake_minimum_required(VERSION 3.8)
project(nrs_path2)

# Set default to Release mode
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##############################
#        FIND PACKAGES       #
##############################
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# ROS 2 core packages
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

# TF and math
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

# Image processing
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)

# MoveIt2
find_package(moveit_ros_planning_interface REQUIRED)

# PCL and related
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

# RealSense
find_package(realsense2 REQUIRED)

# Pluginlib
find_package(pluginlib REQUIRED)

# External libraries
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
find_package(GMP REQUIRED)
find_package(MPFR REQUIRED)
find_package(yaml-cpp REQUIRED)

##############################
#       EXPORT SETTINGS      #
##############################
ament_export_dependencies(
  rclcpp std_msgs sensor_msgs nrs_path2
  image_transport cv_bridge image_geometry
  VTK PCL OpenCV CGAL geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

##############################
#       INCLUDE & LINK       #
##############################
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${MPFR_INCLUDE_DIR}
  ${realsense2_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

set(COMMON_DEPENDENCIES
  rclcpp rclcpp_action std_msgs std_srvs sensor_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs tf2_eigen
  image_transport image_geometry cv_bridge
  pcl_conversions pcl_msgs pluginlib
  moveit_ros_planning_interface realsense2 visualization_msgs
)

set(LINK_LIBS
  ${CGAL_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
  gmp gmpxx mpfr
)

file(GLOB_RECURSE sources "src/*.cpp")

##############################
#   ROS2 MSG/SRV INTERFACE   #
##############################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/Waypoints.msg"
  "srv/Command.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

##############################
#      EXECUTABLES           #
##############################

## path_projection ##
add_executable(path_projection src/path_projection.cpp)
ament_target_dependencies(path_projection ${COMMON_DEPENDENCIES} rosidl_typesupport_cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(path_projection ${LINK_LIBS} ${cpp_typesupport_target})
add_dependencies(path_projection ${PROJECT_NAME}__rosidl_generator_cpp)

## nrs_node_path_generation ##
add_executable(nrs_node_path_generation
  src/nrs_node_path_generation.cpp
  src/nrs_geodesic.cpp
  src/nrs_vec3d.cpp
  src/nrs_interpolation.cpp
  src/nrs_visualization.cpp
  src/nrs_callback.cpp
  src/nrs_io.cpp
  src/nrs_math.cpp
)
ament_target_dependencies(nrs_node_path_generation ${COMMON_DEPENDENCIES} rosidl_typesupport_cpp)
target_link_libraries(nrs_node_path_generation ${LINK_LIBS} ${cpp_typesupport_target})
add_dependencies(nrs_node_path_generation ${PROJECT_NAME}__rosidl_generator_cpp)

## nrs_node_visualization ##
add_executable(nrs_node_visualization
  src/nrs_node_visualization.cpp
  src/nrs_geodesic.cpp
  src/nrs_vec3d.cpp
  src/nrs_interpolation.cpp
  src/nrs_visualization.cpp
  src/nrs_callback.cpp
  src/nrs_io.cpp
  src/nrs_math.cpp
)
ament_target_dependencies(nrs_node_visualization ${COMMON_DEPENDENCIES} rosidl_typesupport_cpp)
target_link_libraries(nrs_node_visualization ${LINK_LIBS} ${cpp_typesupport_target})
add_dependencies(nrs_node_visualization ${PROJECT_NAME}__rosidl_generator_cpp)

##############################
#       INSTALL TARGETS      #
##############################
install(TARGETS
  path_projection
  nrs_node_path_generation
  nrs_node_visualization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY include/ DESTINATION share/${PROJECT_NAME}/include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY mesh/ DESTINATION share/${PROJECT_NAME}/mesh)

##############################
#           TESTS            #
##############################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
