project(Armtraj)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED on)

find_package(Eigen3 REQUIRED HINTS ${CMAKE_SOURCE_DIR}/include/rtde_handarm/Thired_party/Eigen3/install/share/eigen3/cmake)
if(Eigen3_FOUND)
    message(STATUS "Eigen3 Found! - ${Eigen3_DIR}")

    set(Eigen3_LIBS Eigen3::Eigen)
endif()

add_library(Armtraj Armtraj.cpp)
target_include_directories(Armtraj PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(Armtraj PUBLIC ${Eigen3_LIBS})

