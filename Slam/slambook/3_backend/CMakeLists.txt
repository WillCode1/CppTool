cmake_minimum_required(VERSION 2.8)
project(vo)

set(CMAKE_BUILD_TYPE "Release")
# 添加c++ 11标准支持
set(CMAKE_CXX_STANDARD 14)

add_subdirectory(BA)
add_subdirectory(pose_graph)
