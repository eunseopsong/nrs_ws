cmake_minimum_required(VERSION 3.10)
project(nrs_ft_aq2)

# ==================== 필수 패키지 ====================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)  # ✅ std_srvs 추가
find_package(nrs_gen_filter2 REQUIRED)

# ==================== 메시지 인터페이스 정의 ====================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ViveFTMsg.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# ==================== include 디렉토리 ====================
include_directories(include)

# ==================== 라이브러리 생성 ====================
set(SRCS
  src/CAN_reader.cpp
  src/FT_Processing.cpp
)

set(HEADERS
  include/nrs_ft_aq2/CAN_reader.hpp
  include/nrs_ft_aq2/FT_Processing.hpp
)

add_library(ft_lib SHARED ${SRCS} ${HEADERS})
ament_target_dependencies(ft_lib
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs        # ✅ std_srvs 추가
  nrs_gen_filter2
)

# ✅ 메시지 타입 지원 의존성 추가
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_dependencies(ft_lib ${cpp_typesupport_target})

# ==================== 실행 파일 1: CAN_sender ====================
add_executable(can_sender src/CAN_sender.cpp)
ament_target_dependencies(can_sender
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs        # ✅ std_srvs 추가
  nrs_gen_filter2
)
target_link_libraries(can_sender ft_lib)
add_dependencies(can_sender ${cpp_typesupport_target})

# ==================== 실행 파일 2: FT_AQ_main ====================
add_executable(ft_aq_main src/FT_AQ_main.cpp)
ament_target_dependencies(ft_aq_main
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs        # ✅ std_srvs 추가
  nrs_gen_filter2
)
target_link_libraries(ft_aq_main ft_lib)
add_dependencies(ft_aq_main ${cpp_typesupport_target})

# ==================== 설치 ====================
install(TARGETS
  ft_lib
  can_sender
  ft_aq_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# ==================== export ====================
ament_export_dependencies(rosidl_default_runtime)
ament_package()
