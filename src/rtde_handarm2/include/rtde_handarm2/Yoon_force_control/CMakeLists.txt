project(Yoon_force_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED on)

find_package(Eigen3 REQUIRED HINTS ${CMAKE_SOURCE_DIR}/Thired_party/Eigen3/install/share/eigen3/cmake)
if(Eigen3_FOUND)
    message(STATUS "Eigen3 Found! - ${Eigen3_DIR}")

    set(Eigen3_LIBS Eigen3::Eigen)
endif()

set(UR10_dir ${CMAKE_SOURCE_DIR}/rtde_handarm/include/rtde_handarm/UR10)

add_library(Yoon_force_control Yoon_force_control.cpp )
target_include_directories(Yoon_force_control PUBLIC ${Eigen3_INCLUDE_DIRS} ${UR10_dir})
target_link_libraries(Yoon_force_control ${Eigen3_LIBS} Kinematics)