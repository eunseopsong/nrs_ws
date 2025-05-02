include_directories("./")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${MPFR_INCLUDE_DIR}
  ${realsense2_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)

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
"./*.cpp"
)

# By doing this, we can add all the source files in the current directory and its subdirectories to the executable
# add_executable(nrs_node_path_generation ${sources})

#######################################################
#### nrs_node_path_generation dependencies setting ####
#######################################################

add_executable(nrs_node_path_generation
# src/nrs_node_path_generation.cpp

# src/nrs_geodesic.cpp
# src/nrs_vec3d.cpp
# src/nrs_interpolation.cpp
# src/nrs_visualization.cpp


nrs_callback.cpp    # header file Immigration is done
# nrs_io.cpp          # header file Immigration is done
# nrs_math.cpp
)

# It is not needed if you don't use the include directory
target_include_directories(nrs_node_path_generation PUBLIC include)

ament_target_dependencies(nrs_node_path_generation ${COMMON_DEPENDENCIES}) # add_dependencies(nrs_node_path_generation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})









