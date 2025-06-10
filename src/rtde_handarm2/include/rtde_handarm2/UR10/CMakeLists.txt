# project(UR10)

# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED on)

# find_package(Eigen3 REQUIRED HINTS ${CMAKE_SOURCE_DIR}/rtde_handarm/Thired_party/Eigen3/install/share/eigen3/cmake)
# if(Eigen3_FOUND)
#     message(STATUS "Eigen3 Found! - ${Eigen3_DIR}")

#     set(Eigen3_LIBS Eigen3::Eigen)
# endif()

# # ------- Yaml packages -------- #
# find_package(yaml-cpp REQUIRED)
# if(yaml-cpp_FOUND)
#   message(STATUS "yaml-cpp Found! - ${yaml-cpp_DIR}")
# endif()
# # -------------------------------#

# set(NRS_yaml_dir ${CMAKE_SOURCE_DIR}/rtde_handarm/NRS_yaml)


# add_library(Kinematics Kinematics.cpp)
# target_include_directories(Kinematics PRIVATE ${Eigen3_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR} ${NRS_yaml_dir})
# target_link_libraries(Kinematics PRIVATE ${Eigen3_LIBS} ${YAML_CPP_LIBRARIES})