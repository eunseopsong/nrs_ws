cmake_minimum_required(VERSION 3.8)
project(Y2ForceCon)

# =========================================================
# [CHANGED] LibTorch와 ABI/컴파일 옵션 충돌 방지 위해 명시
# =========================================================
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# =========================================================
# Torch
# =========================================================

# [CHANGED] ENV{LIBTORCH}로 Torch_DIR 자동 설정
if(NOT Torch_DIR AND DEFINED ENV{LIBTORCH})
  set(Torch_DIR "$ENV{LIBTORCH}/share/cmake/Torch")
endif()

find_package(Torch REQUIRED)

# [CHANGED] Torch가 요구하는 컴파일 플래그 적용(ABI/경고 관련)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# [CHANGED] LibTorch root 추정(= $LIBTORCH)
set(_LIBTORCH "")
if(DEFINED ENV{LIBTORCH})
  set(_LIBTORCH "$ENV{LIBTORCH}")
elseif(DEFINED Torch_DIR)
  get_filename_component(_LIBTORCH "${Torch_DIR}/../../.." ABSOLUTE)
endif()

# [CHANGED] 런타임에 libtorch/lib 경로를 자동으로 찾도록 RPATH 설정
if(_LIBTORCH)
  set(CMAKE_BUILD_RPATH "${_LIBTORCH}/lib")
  set(CMAKE_INSTALL_RPATH "${_LIBTORCH}/lib")
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

# [CHANGED] include / link 목록 구성 (Torch::Torch 있으면 우선 사용, 없으면 TORCH_LIBRARIES 사용)
set(TORCH_INC ${TORCH_INCLUDE_DIRS})
if(TARGET Torch::Torch)
  set(TORCH_LINK Torch::Torch)
else()
  set(TORCH_LINK ${TORCH_LIBRARIES})
endif()

# =========================================================
# [CHANGED] MKL undefined reference 해결 핵심:
#   libtorch_cpu.so가 요구하는 deps를 torch_global_deps가 들고 있는 케이스가 많음
#   (존재하면 자동으로 같이 링크)
# =========================================================
if(_LIBTORCH AND EXISTS "${_LIBTORCH}/lib/libtorch_global_deps.so")
  list(APPEND TORCH_LINK "${_LIBTORCH}/lib/libtorch_global_deps.so")
endif()

# 디버그 출력
message(STATUS "Torch_DIR='${Torch_DIR}'")
message(STATUS "_LIBTORCH='${_LIBTORCH}'")
message(STATUS "TORCH_LIBRARIES='${TORCH_LIBRARIES}'")
message(STATUS "TORCH_INCLUDE_DIRS='${TORCH_INCLUDE_DIRS}'")
message(STATUS "TORCH_LINK(final)='${TORCH_LINK}'")

# =========================================================
# Libraries
# =========================================================

add_library(Yadmittance_control src/admittance_control.cpp)
target_include_directories(Yadmittance_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(Yadmittance_control rclcpp)

add_library(nrs_3step_faac src/nrs_3step_faac.cpp)
target_include_directories(nrs_3step_faac PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(nrs_3step_faac rclcpp)

add_library(rl_mass_variation src/rl_mass_variation.cpp)
# [CHANGED] TORCH_INC 포함
target_include_directories(rl_mass_variation SYSTEM PUBLIC
  ${TORCH_INC}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rl_mass_variation rclcpp)
# [CHANGED] ament와 충돌 방지: target_link_libraries는 "plain signature"만 사용
target_link_libraries(rl_mass_variation ${TORCH_LINK})

add_library(rl_md_variation src/rl_md_variation.cpp)
# [CHANGED] TORCH_INC 포함
target_include_directories(rl_md_variation SYSTEM PUBLIC
  ${TORCH_INC}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rl_md_variation rclcpp)
# [CHANGED] ament와 충돌 방지: target_link_libraries는 "plain signature"만 사용
target_link_libraries(rl_md_variation ${TORCH_LINK})

# =========================================================
# Executables
# =========================================================

add_executable(ACexample src/examples/ACexample.cpp)
ament_target_dependencies(ACexample rclcpp)
target_link_libraries(ACexample Yadmittance_control)

add_executable(rl_mass_example src/examples/rl_mass_example.cpp)
ament_target_dependencies(rl_mass_example rclcpp)
# [CHANGED] (STATIC 라이브러리/링크 순서 대비) exe에도 TORCH_LINK를 한 번 더 걸어줌
target_link_libraries(rl_mass_example rl_mass_variation ${TORCH_LINK})

add_executable(rl_md_example src/examples/rl_md_example.cpp)
ament_target_dependencies(rl_md_example rclcpp)
# [CHANGED] (STATIC 라이브러리/링크 순서 대비) exe에도 TORCH_LINK를 한 번 더 걸어줌
target_link_libraries(rl_md_example rl_md_variation ${TORCH_LINK})

# =========================================================
# Install
# =========================================================

install(TARGETS
  Yadmittance_control
  nrs_3step_faac
  rl_mass_variation
  rl_md_variation
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS
  ACexample
  rl_mass_example
  rl_md_example
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY src/checkpoints/
  DESTINATION share/${PROJECT_NAME}/checkpoints
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
