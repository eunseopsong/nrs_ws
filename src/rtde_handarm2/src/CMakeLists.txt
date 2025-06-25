include_directories("./")
# include_directories("./Controllers")
# include_directories("./FSM_States")

# include_directories("../../third-party/")
# include_directories(SYSTEM "../../third-party/qpOASES/include")
# include_directories("../../third-party/ParamHandler")



# include_directories(
#   ${GAZEBO_INCLUDE_DIRS}
#   ${qpOASES_INCLUDE_DIR}
#   ${PROJECT_SOURCE_DIR}
#   ${CMAKE_BINARY_DIR}
# )

file(GLOB_RECURSE sources
# "./*.cpp"
# "FSM_States/*.cpp"
# "Controllers/convexMPC/*.cpp"
)

add_executable(aidin_ctrl ${sources} ur10e_main.cpp)

# Link the necessary libraries to the executable
# target_link_libraries(aidin_ctrl
#   ${GAZEBO_LIBRARIES}
#   ${rclcpp_LIBRARIES}
#   ${qpOASES_LIBRARIES}
# )

# Optionally install the executable
# install(TARGETS aidin_ctrl
#   DESTINATION lib/${PROJECT_NAME}
# )

# add_subdirectory(Controllers/WBC)
# add_subdirectory(Controllers/WBC_Ctrl)
# add_subdirectory(Controllers/VisionMPC)

# add_executable(aidin_ctrl ${sources} AiDIN_Controller.cpp main.cpp)
# target_link_libraries(mit_ctrl robot biomimetics)
# target_link_libraries(aidin_ctrl dynacore_param_handler qpOASES)
# target_link_libraries(mit_ctrl Goldfarb_Optimizer osqp)
# target_link_libraries(mit_ctrl WBC_Ctrl)
# target_link_libraries(mit_ctrl VisionMPC)