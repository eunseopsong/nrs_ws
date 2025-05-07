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
# find_package(eigen_conversions REQUIRED)

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
# find_package(CGAL REQUIRED COMPONENTS Core)

####################################
##### catkin 패키지 설정 -> ROS2 #####
####################################
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
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


###################################
## include 디렉토리 설정 ## (nrs_path 그대로 사용; 나중에 문제 생기면 수정)
###################################
include_directories(
  # ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${MPFR_INCLUDE_DIR}
  ${realsense2_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

set(COMMON_DEPENDENCIES
  rclcpp
  rclcpp_action
  geometry_msgs
  std_srvs
  std_msgs
  sensor_msgs

  # message_generation

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



## 서비스 파일 추가
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Waypoint.msg"
#   "msg/Waypoints.msg"
#   "srv/Command.srv"
#   DEPENDENCIES
#   geometry_msgs
#   std_msgs
# )


####### dependencies of path_projection.cpp (start) #######

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoints.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(path_projection src/path_projection.cpp)

# ROS 2 및 외부 의존성 설정
ament_target_dependencies(path_projection ${COMMON_DEPENDENCIES})

target_include_directories(path_projection PUBLIC
  ${CGAL_INCLUDE_DIRS}
)

# 외부 라이브러리 직접 링크
target_link_libraries(path_projection
  ${CGAL_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
)

###### dependencies of path_projection.cpp (end) #######


# nrs_node_path_generation dependencies setting
add_executable(nrs_node_path_generation ${sources}) # By doing this, we can add all the source files in the current directory and its subdirectories to the executable
ament_target_dependencies(nrs_node_path_generation ${COMMON_DEPENDENCIES}) # add_dependencies(nrs_node_path_generation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# nrs_node_visualization dependencies setting
add_executable(nrs_node_visualization ${sources})
ament_target_dependencies(nrs_node_visualization ${COMMON_DEPENDENCIES})

# nrs_node_simulation dependencies setting
# add_executable(nrs_node_simulation ${sources})
# ament_target_dependencies(nrs_node_simulation ${COMMON_DEPENDENCIES})






if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}/include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY mesh/
  DESTINATION share/${PROJECT_NAME}/mesh
)

# include the sub directory
# add_subdirectory(src)

ament_package()
