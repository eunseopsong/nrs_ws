# cmake_minimum_required(VERSION 3.10)
# project(nrs_ft_aq2)

# # ========================
# # Dependencies
# # ========================
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# # find_package(NRS_Gen_Filter REQUIRED)

# # If you have custom messages
# find_package(rosidl_default_generators REQUIRED)

# # ========================
# # Message Files
# # ========================
# set(msg_files
#   "msg/vive_ft_msg.msg"
# )

# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
#   DEPENDENCIES std_msgs geometry_msgs
# )

# # ========================
# # Include & Sources
# # ========================
# set(SRCS
#   src/CAN_reader.cpp
#   src/FT_Processing.cpp
# )

# set(HEADERS
#   include/CAN_reader.hpp
#   include/FT_Processing.hpp
# )

# include_directories(
#   include
# )

# # ========================
# # Library
# # ========================
# add_library(FT_lib ${SRCS} ${HEADERS})
# ament_target_dependencies(FT_lib rclcpp std_msgs geometry_msgs NRS_Gen_Filter)

# # ========================
# # Executables
# # ========================
# add_executable(CAN_sender src/CAN_sender.cpp)
# ament_target_dependencies(CAN_sender rclcpp std_msgs geometry_msgs NRS_Gen_Filter)

# add_executable(FT_AQ_main src/FT_AQ_main.cpp)
# target_link_libraries(FT_AQ_main
#   FT_lib
# )
# ament_target_dependencies(FT_AQ_main rclcpp std_msgs geometry_msgs NRS_Gen_Filter)

# # Add message dependencies
# rosidl_target_interfaces(FT_AQ_main
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )

# # ========================
# # Install Targets
# # ========================
# install(TARGETS
#   FT_lib
#   CAN_sender
#   FT_AQ_main
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY include/
#   DESTINATION include/
# )

# install(DIRECTORY msg
#   DESTINATION share/${PROJECT_NAME}/
# )

# # ========================
# # Export
# # ========================
# ament_export_include_directories(include)
# ament_export_dependencies(rclcpp std_msgs geometry_msgs NRS_Gen_Filter)
# ament_package()
