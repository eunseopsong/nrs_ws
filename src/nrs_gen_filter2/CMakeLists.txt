cmake_minimum_required(VERSION 3.10)
project(nrs_gen_filter2)

# ========================
# Dependencies
# ========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# ========================
# Include directories
# ========================
# 다른 package에서 find package하면 내 include directory 역시 
# 자동으로 할당하게 하기 위함
# 만들어 놓은 library 역시 마찬가지
include_directories(include)

# ========================
# Library 1: Gen Filter Core
# ========================
add_library(nrs_gen_filter_lib
  src/Gen_filter.cpp
)
ament_target_dependencies(nrs_gen_filter_lib rclcpp)

# ========================
# Library 2: Filter Applied Logic (includes Gen_filter.cpp again)
# ========================
add_library(nrs_filter_applied_lib
  src/Filter_applied.cpp
  src/Gen_filter.cpp
)
ament_target_dependencies(nrs_filter_applied_lib rclcpp)

# ========================
# Installation
# ========================
install(
  TARGETS
    nrs_gen_filter_lib
    nrs_filter_applied_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# ========================
# Export
# ========================
ament_export_include_directories(include)
ament_export_libraries(
  nrs_gen_filter_lib
  nrs_filter_applied_lib
)
ament_export_dependencies(rclcpp)

ament_package()
