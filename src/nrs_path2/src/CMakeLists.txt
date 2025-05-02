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

file(GLOB_RECURSE sources
"./*.cpp"
)

# By doing this, we can add all the source files in the current directory and its subdirectories to the executable
# add_executable(nrs_node_path_generation ${sources})


add_executable(nrs_node_path_generation
# src/nrs_node_path_generation.cpp

# src/nrs_geodesic.cpp
# src/nrs_vec3d.cpp
# src/nrs_interpolation.cpp
# src/nrs_visualization.cpp


nrs_callback.cpp
# src/nrs_io.cpp
# src/nrs_math.cpp
)

# add_dependencies(nrs_node_path_generation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})