cmake_minimum_required(VERSION 3.8)
project(nrs_ft_aq2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1) msg 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ViveFtMsg.msg"
  DEPENDENCIES std_msgs
)

# 2) include 설치만
install(DIRECTORY include/
  DESTINATION include/
)

# 3) (지금은 src가 없으니까 빌드 타깃 안 만든다)
# 나중에 src/CAN_reader.cpp 같은 거 생기면 여기서 add_library/add_executable 추가하면 됨.

ament_package()
