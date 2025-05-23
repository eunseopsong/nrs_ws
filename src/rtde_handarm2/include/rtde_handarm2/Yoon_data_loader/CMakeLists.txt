project(Yoon_data_loader)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED on)

find_package(Eigen3 REQUIRED HINTS ${CMAKE_SOURCE_DIR}/Thired_party/Eigen3/install/share/eigen3/cmake)
if(Eigen3_FOUND)
    message(STATUS "Eigen3 Found! - ${Eigen3_DIR}")

    set(Eigen3_LIBS Eigen3::Eigen)
endif()


add_library(Text_loader Text_loader.cpp)
target_include_directories(Text_loader PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(Text_loader ${Eigen3_LIBS})
