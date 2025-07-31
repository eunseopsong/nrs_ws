cmake_minimum_required(VERSION 3.10)
project(NRS_Gen_Filter)

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
add_library(NRS_Gen_Filter_Lib
  src/Gen_filter.cpp
)
ament_target_dependencies(NRS_Gen_Filter_Lib rclcpp)

# ========================
# Library 2: Filter Applied Logic (includes Gen_filter.cpp again)
# ========================
add_library(NRS_Filter_Applied_Lib
  src/Filter_applied.cpp
  src/Gen_filter.cpp
)
ament_target_dependencies(NRS_Filter_Applied_Lib rclcpp)

# ========================
# Installation
# ========================
install(
  TARGETS
    NRS_Gen_Filter_Lib
    NRS_Filter_Applied_Lib
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
  NRS_Gen_Filter_Lib
  NRS_Filter_Applied_Lib
)
ament_export_dependencies(rclcpp)

ament_package()
