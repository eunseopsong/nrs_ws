cmake_minimum_required(VERSION 3.8)
project(Y2FT_AQ)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_python REQUIRED)   # ❌ 필요 없으면 지워도 됨
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Y2Filters REQUIRED)
find_package(Y2Matrix REQUIRED)
find_package(ament_index_cpp REQUIRED)        # ✅ log 경로용

include_directories(include)

# === Create libraries ===
add_library(FT_EtherGet src/FT_EtherGet.cpp)
target_include_directories(FT_EtherGet PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>   # CAMKE → CMAKE 로 오타 수정
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(FT_EtherGet rclcpp Y2Matrix)

add_library(FT_eCANGet src/FT_eCANGet.cpp)
target_include_directories(FT_eCANGet PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(FT_eCANGet rclcpp Y2Matrix)

# === Create executables ===
add_executable(FT_EtherGet_Example examples/FT_EtherGet_Example.cpp)
target_link_libraries(FT_EtherGet_Example FT_EtherGet)
ament_target_dependencies(FT_EtherGet_Example rclcpp)

add_executable(FT_eCANSend src/FT_eCANSend.cpp)
ament_target_dependencies(FT_eCANSend rclcpp)

add_executable(FTGetMain src/FTGetMain.cpp)
target_link_libraries(FTGetMain FT_EtherGet FT_eCANGet)
ament_target_dependencies(FTGetMain
  rclcpp
  Y2Filters
  geometry_msgs
  std_msgs
  std_srvs
  Y2Matrix
  ament_index_cpp
)

# === Create install ===
install(TARGETS FT_EtherGet FT_eCANGet
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS FT_EtherGet_Example FTGetMain FT_eCANSend
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# === Install Python scripts only ===

install(PROGRAMS
  scripts/G_Comp_Matrix.py        # * 대신 이렇게 명시해 두는게 깔끔
  DESTINATION lib/${PROJECT_NAME}
)

# === Export targets & dependencies === 
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs std_srvs geometry_msgs Y2Filters Y2Matrix)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
