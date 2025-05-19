cmake_minimum_required(VERSION 3.8)
project(nrs_forcecon2)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 종속성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# include 디렉토리 등록
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# 라이브러리 생성
add_library(nrs_3step_faac src/nrs_3step_faac.cpp)
ament_target_dependencies(nrs_3step_faac
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  Eigen3
)

# 실행 노드: kf_acc_tester
# add_executable(kf_acc_tester src/kf_acc_tester.cpp)
# target_link_libraries(kf_acc_tester
#   nrs_3step_faac
# )
# ament_target_dependencies(kf_acc_tester
#   rclcpp
#   std_msgs
#   geometry_msgs
#   sensor_msgs
#   Eigen3
# )

# 설치 대상 설정
install(TARGETS
  # nrs_3step_faac
  # kf_acc_tester
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# 패키지 선언
ament_package()
