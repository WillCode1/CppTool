cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")
# 添加c++ 11标准支持
set(CMAKE_CXX_STANDARD 14)

add_subdirectory(sophus)
add_subdirectory(eigen)
add_subdirectory(g2o)
