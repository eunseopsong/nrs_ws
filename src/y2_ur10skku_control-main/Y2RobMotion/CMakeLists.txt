cmake_minimum_required(VERSION 3.8)
project(Y2RobMotion)

set(CMAKE_CXX_STANDARD 14)                 # [CHANGED]
set(CMAKE_CXX_STANDARD_REQUIRED ON)        # [CHANGED]
set(CMAKE_POSITION_INDEPENDENT_CODE ON)    # [CHANGED] (DT_TEXTREL/PIE 경고 완화)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ======================
# find dependencies
# ======================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)            # [CHANGED] robot_measure에서 사용

find_package(Y2Matrix REQUIRED)
find_package(Y2Kinematics REQUIRED)
find_package(Y2Trajectory REQUIRED)
find_package(Y2ForceCon REQUIRED)
find_package(osqp REQUIRED)

set(AMENT_DEP
  rclcpp sensor_msgs geometry_msgs std_msgs   # [CHANGED] std_msgs 추가
  Y2Matrix Y2Kinematics Y2Trajectory Y2ForceCon
)

include_directories(include)

# =========================================================
# [CHANGED] Torch (명시 링크: Y2ForceCon 정적라이브러리의 torch 심볼 해결)
# =========================================================
if(NOT Torch_DIR AND DEFINED ENV{LIBTORCH})
  set(Torch_DIR "$ENV{LIBTORCH}/share/cmake/Torch")
endif()
find_package(Torch REQUIRED)

# libtorch root 추정
set(_LIBTORCH "")
if(DEFINED ENV{LIBTORCH})
  set(_LIBTORCH "$ENV{LIBTORCH}")
elseif(EXISTS "$ENV{HOME}/libtorch")
  set(_LIBTORCH "$ENV{HOME}/libtorch")
endif()

# include
set(TORCH_INC ${TORCH_INCLUDE_DIRS})

# link libs (네 배포본이 Torch::Torch 없을 수 있어서 절대경로로 강제)
set(TORCH_LINK "")
if(_LIBTORCH)
  # 링크 검색 경로 / rpath
  link_directories("${_LIBTORCH}/lib")                       # [CHANGED]
  set(CMAKE_BUILD_RPATH   "${_LIBTORCH}/lib")                # [CHANGED]
  set(CMAKE_INSTALL_RPATH "${_LIBTORCH}/lib")                # [CHANGED]
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)                # [CHANGED]

  # torch_library 토큰 방지: 절대경로로 강제 링크
  list(APPEND TORCH_LINK
    "${_LIBTORCH}/lib/libtorch.so"
    "${_LIBTORCH}/lib/libtorch_cpu.so"
    "${_LIBTORCH}/lib/libc10.so"
  )

  if(EXISTS "${_LIBTORCH}/lib/libtorch_global_deps.so")      # [CHANGED] 있으면 같이
    list(APPEND TORCH_LINK "${_LIBTORCH}/lib/libtorch_global_deps.so")
  endif()

  # 필요 시 런타임용 심링크(예전 -ltorch_library 대응)
  if(EXISTS "${_LIBTORCH}/lib/libtorch.so" AND NOT EXISTS "${_LIBTORCH}/lib/libtorch_library.so")
    file(CREATE_LINK
      "${_LIBTORCH}/lib/libtorch.so"
      "${_LIBTORCH}/lib/libtorch_library.so"
      SYMBOLIC
    )
  endif()
else()
  message(FATAL_ERROR "LIBTORCH not found. export LIBTORCH=/home/nrs_robot/libtorch")
endif()

message(STATUS "TORCH_INC='${TORCH_INC}'")                   # [CHANGED]
message(STATUS "TORCH_LINK='${TORCH_LINK}'")                 # [CHANGED]

# ======================
# sources
# ======================
set(MAIN_SOURCES
  src/robot_motion.cpp
  src/hand_guiding.cpp
  src/force_control.cpp
  src/state_monitoring.cpp
)

# ======================
# libraries
# ======================
add_library(robot_motion ${MAIN_SOURCES})
target_include_directories(robot_motion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TORCH_INC}                                              # [CHANGED]
)
ament_target_dependencies(robot_motion ${AMENT_DEP})
target_link_libraries(robot_motion ${TORCH_LINK})           # [CHANGED] (ament_target_dependencies가 plain이라 plain 유지)

add_library(robot_command src/robot_command.cpp)
target_include_directories(robot_command PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TORCH_INC}                                              # [CHANGED] (안 넣어도 되지만 안전하게)
)
ament_target_dependencies(robot_command ${AMENT_DEP})
target_link_libraries(robot_command ${TORCH_LINK})          # [CHANGED]

# ======================
# executables
# ======================
add_executable(singleArm_motion src/singleArm_motion.cpp)
target_link_libraries(singleArm_motion robot_motion ${TORCH_LINK})  # [CHANGED] 최종 링크에도 torch 강제

add_executable(singleArm_cmd src/singleArm_cmd.cpp)
target_link_libraries(singleArm_cmd robot_command ${TORCH_LINK})    # [CHANGED]

add_executable(robot_measure src/robot_measure.cpp)
ament_target_dependencies(robot_measure ${AMENT_DEP})
target_link_libraries(robot_measure ${TORCH_LINK})                  # [CHANGED] 혹시 몰라서 torch 강제

# ======================
# install
# ======================
install(TARGETS robot_motion robot_command
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS singleArm_motion singleArm_cmd robot_measure
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AMENT_DEP})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
