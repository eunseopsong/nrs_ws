cmake_minimum_required(VERSION 3.8)
project(nrs_path2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################# find dependencies #################
find_package(ament_cmake REQUIRED)

# Basic ROS2 packages
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED) # for Service and Message generation

# about TF and Eigen
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

# about Image
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)

# about MoveIt2
find_package(moveit_ros_planning_interface REQUIRED)

# about PCL
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

# about Intel RealSense
find_package(realsense2 REQUIRED)

# about pluginlib
find_package(pluginlib REQUIRED)

############ find external library (PCL, OpenCV, CGAL, VTK, GMP, MPFR) ############
find_package(VTK REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)

ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  nrs_path2
  image_transport
  cv_bridge
  image_geometry
  VTK
  PCL
  OpenCV
  CGAL
  geometry_msgs
)

ament_export_include_directories(
  include
)

ament_export_dependencies(rosidl_default_runtime)

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
  rclcpp
  rclcpp_action
  geometry_msgs
  std_srvs
  std_msgs
  sensor_msgs
  cv_bridge
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  image_transport
  image_geometry
  pcl_msgs
  pcl_conversions
  pluginlib
  moveit_ros_planning_interface
  realsense2
)

file(GLOB_RECURSE sources
"src/*.cpp"
)

####### Message and Service Interface Generation #######
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/Waypoints.msg"
  "srv/Command.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

####### Executable Definitions #######
add_executable(path_projection src/path_projection.cpp)
ament_target_dependencies(path_projection ${COMMON_DEPENDENCIES})
target_include_directories(path_projection PUBLIC
  ${CGAL_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
add_dependencies(path_projection ${PROJECT_NAME}__rosidl_generator_cpp)
target_link_libraries(path_projection
  ${CGAL_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
)

#### nrs_node_path_generation ####
# add_executable(nrs_node_path_generation ${sources})
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

ament_target_dependencies(nrs_node_path_generation ${COMMON_DEPENDENCIES})
target_include_directories(nrs_node_path_generation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
add_dependencies(nrs_node_path_generation ${PROJECT_NAME}__rosidl_generator_cpp)


#### nrs_node_visualization ####
# add_executable(nrs_node_visualization ${sources})
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

ament_target_dependencies(nrs_node_visualization ${COMMON_DEPENDENCIES})
target_include_directories(nrs_node_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
add_dependencies(nrs_node_visualization ${PROJECT_NAME}__rosidl_generator_cpp)


#### nrs_node_simulation ####
# add_executable(nrs_node_simulation ${sources})
# ament_target_dependencies(nrs_node_simulation ${COMMON_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY include/ DESTINATION share/${PROJECT_NAME}/include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY mesh/ DESTINATION share/${PROJECT_NAME}/mesh)

ament_package()