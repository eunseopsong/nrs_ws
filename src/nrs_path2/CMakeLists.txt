cmake_minimum_required(VERSION 3.8)
project(nrs_path2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


################# find dependencies {ROS 필수 패키지 찾기} #################
find_package(ament_cmake REQUIRED)

# Basic ROS2 packages
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_srvs REQUIRED)


# about TF and Eigen
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(eigen_conversions REQUIRED)
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


############ 외부 라이브러리 찾기 (PCL, OpenCV, CGAL, VTK, GMP, MPFR) ############

find_package(VTK REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

find_package(CGAL REQUIRED)
# find_package(CGAL REQUIRED COMPONENTS Core)


## 서비스 파일 추가
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/Waypoints.msg"
  "srv/Command.srv"
  DEPENDENCIES
  geometry_msgs
  std_msgs
)


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
  include
  /usr/local/include
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

link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

###################################
## 빌드 설정 (ROS 2 스타일) ##
###################################

add_executable(path_projection src/path_projection.cpp)

target_include_directories(path_projection PUBLIC
  # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CGAL_INCLUDE_DIRS}
)

# ROS 2 및 외부 의존성 설정
ament_target_dependencies(path_projection
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

# 외부 라이브러리 직접 링크
target_link_libraries(path_projection
  ${CGAL_LIBRARIES}

  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
)

# (선택) include 디렉토리 직접 추가 시
# target_include_directories(path_projection PUBLIC
#   include
#   /usr/local/include
#   ${EIGEN3_INCLUDE_DIRS}
#   ${VTK_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
#   ${CGAL_INCLUDE_DIRS}
#   ${MPFR_INCLUDE_DIR}
#   ${realsense2_INCLUDE_DIR}
#   ${YAML_CPP_INCLUDE_DIRS}
# )






install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY mesh/
  DESTINATION share/${PROJECT_NAME}/mesh
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)






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
