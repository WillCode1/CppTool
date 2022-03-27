cmake_minimum_required(VERSION 2.8)
project(SlamBook)

set(CMAKE_BUILD_TYPE "Release")
# 添加c++ 11标准支持
set(CMAKE_CXX_STANDARD 14)

add_subdirectory(math_lib)
add_subdirectory(camera)
add_subdirectory(tool)
add_subdirectory(vo)
