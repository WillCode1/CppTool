cmake_minimum_required(VERSION 2.8)
project(SlamBook)

set(CMAKE_BUILD_TYPE "Release")
# 添加c++ 11标准支持
set(CMAKE_CXX_STANDARD 14)

add_subdirectory(1_camera)
add_subdirectory(2_vo)
add_subdirectory(3_backend)
add_subdirectory(4_loop_closure)
add_subdirectory(5_mapping)
add_subdirectory(6_my_slam)
add_subdirectory(math_lib)
add_subdirectory(tool)
