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


####### dependencies of path_projection.cpp (start) #######

# add_executable(path_projection path_projection.cpp)

# target_include_directories(path_projection PUBLIC
#   ${CGAL_INCLUDE_DIRS}
# )

# # ROS 2 및 외부 의존성 설정
# ament_target_dependencies(nrs_node_visualization ${COMMON_DEPENDENCIES})

# # 외부 라이브러리 직접 링크
# target_link_libraries(path_projection
#   ${CGAL_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${YAML_CPP_LIBRARIES}
# )

###### dependencies of path_projection.cpp (end) #######


# nrs_node_path_generation dependencies setting
add_executable(nrs_node_path_generation ${sources}) # By doing this, we can add all the source files in the current directory and its subdirectories to the executable

# add_executable(nrs_node_path_generation
# nrs_node_path_generation.cpp

# nrs_geodesic.cpp
# nrs_vec3d.cpp
# nrs_interpolation.cpp
# nrs_visualization.cpp

# nrs_callback.cpp
# nrs_io.cpp
# nrs_math.cpp
# )

# It is not needed if you don't use the include directory
# target_include_directories(nrs_node_path_generation PUBLIC include)

ament_target_dependencies(nrs_node_path_generation ${COMMON_DEPENDENCIES}) # add_dependencies(nrs_node_path_generation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# nrs_node_visualization dependencies setting
add_executable(nrs_node_visualization ${sources})
ament_target_dependencies(nrs_node_visualization ${COMMON_DEPENDENCIES})

# nrs_node_simulation dependencies setting
# add_executable(nrs_node_simulation ${sources})
# ament_target_dependencies(nrs_node_simulation ${COMMON_DEPENDENCIES})




