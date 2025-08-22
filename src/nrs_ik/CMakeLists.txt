cmake_minimum_required(VERSION 3.10)
project(nrs_ik)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include dirs
# 헤더들이 src 폴더에 있으니 src도 include path에 추가
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
)

# ------------------------------
# UR10e kinematics library
# ------------------------------
# 반드시 src/Kinematics.cpp 파일을 만들어서(너가 보낸 구현 붙여넣기) 컴파일되게 해줘.
# Arm_class.cpp가 따로 있으면 아래에 추가. (없으면 지워도 됨)
set(UR10E_KIN_SRC
  src/Kinematics.cpp
  # src/Arm_class.cpp
)

add_library(ur10e_kinematics ${UR10E_KIN_SRC})
target_link_libraries(ur10e_kinematics Eigen3::Eigen)
# rclcpp/sensor_msgs에 직접 의존하지는 않지만, 다른 타겟과의 일관성을 위해 ament deps는 생략 가능

target_include_directories(ur10e_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

# ------------------------------
# Executables
# ------------------------------
add_executable(inverse_kinematics_node src/inverse_kinematics_node.cpp)
ament_target_dependencies(inverse_kinematics_node rclcpp std_msgs sensor_msgs)
target_link_libraries(inverse_kinematics_node ur10e_kinematics)

add_executable(forward_kinematics_node src/forward_kinematics_node.cpp)
ament_target_dependencies(forward_kinematics_node rclcpp std_msgs)
target_link_libraries(forward_kinematics_node ur10e_kinematics)

add_executable(txt_trajectory_executor src/txt_trajectory_executor.cpp)
ament_target_dependencies(txt_trajectory_executor rclcpp sensor_msgs)
# 이 노드는 직접 IK 안 쓰면 링크 불필요. 필요하면 아래 줄 주석 해제
# target_link_libraries(txt_trajectory_executor ur10e_kinematics)

add_executable(file_reader_node src/file_reader_node.cpp)
ament_target_dependencies(file_reader_node rclcpp)

add_executable(txt_ik_executor src/txt_ik_executor.cpp)
ament_target_dependencies(txt_ik_executor rclcpp sensor_msgs)
target_link_libraries(txt_ik_executor ur10e_kinematics)

# ------------------------------
# Install
# ------------------------------
install(TARGETS
  ur10e_kinematics
  inverse_kinematics_node
  forward_kinematics_node
  txt_trajectory_executor
  file_reader_node
  txt_ik_executor
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 설치 (현재 src 밑에 있으니, 배포하려면 include로 설치 경로 바꿔줌)
install(FILES
  src/Kinematics.h
  src/Arm_class.h
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
