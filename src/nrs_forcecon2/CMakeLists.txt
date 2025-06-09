cmake_minimum_required(VERSION 3.8)
project(nrs_forcecon2)

# — C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# — find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(std_msgs     REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs  REQUIRED)
find_package(Eigen3       REQUIRED)

# — include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# — libraries
add_library(nrs_3step_faac
  src/nrs_3step_faac.cpp
)
ament_target_dependencies(nrs_3step_faac
  rclcpp std_msgs geometry_msgs sensor_msgs Eigen3
)

# — test 실행 파일 (이제 라이브러리가 아니고, 그대로 executable)
add_executable(kf_acc_tester src/kf_acc_tester.cpp)
target_link_libraries(kf_acc_tester
  nrs_3step_faac
)
ament_target_dependencies(kf_acc_tester
  rclcpp std_msgs geometry_msgs sensor_msgs Eigen3
)

# — install targets
install(
  TARGETS
    nrs_3step_faac
    kf_acc_tester
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

# — export library targets
#    * 여기서 executable(kf_acc_tester)은 export 하지 않습니다!
ament_export_libraries(nrs_3step_faac)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs geometry_msgs sensor_msgs Eigen3)

install(
  EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package()
