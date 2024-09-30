cmake_minimum_required(VERSION 3.0)
PROJECT (parallel_forward_kinematics)

# Set the C++ standard to C++11
add_compile_options(-O3 -std=c++11)

find_package(RBDL REQUIRED)
find_package(Eigen3 3.0.0 REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    )

add_executable(para_fk src/main.cpp)
target_include_directories(para_fk PUBLIC
    ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(para_fk
    ${RBDL_LIBRARY}
    ${RBDL_URDFREADER_LIBRARY}
    pthread
    )