cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")
# 添加c++ 11标准支持
set(CMAKE_CXX_STANDARD 14)

add_subdirectory(visualizeGeometry)
add_subdirectory(plotTrajectory)
add_subdirectory(trajectoryError)
