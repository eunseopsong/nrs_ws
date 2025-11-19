cmake_minimum_required(VERSION 3.8)
project(urp_cvt_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

set(SRV_FILES
  "srv/GenerateCVTSamples.srv"
)

message(STATUS "CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")
foreach(_f IN LISTS SRV_FILES)
  message(STATUS "Checking SRV file: ${_f} -> ${CMAKE_CURRENT_SOURCE_DIR}/${_f}")
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
