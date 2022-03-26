cmake_minimum_required(VERSION 2.8)
project(useEigen)

set(CMAKE_CXX_STANDARD 14)

include_directories("/usr/include/eigen3")
add_executable(coordinateTransform coordinateTransform.cpp)
add_executable(eigenGeometry eigenGeometry.cpp)
add_executable(eigenMatrix eigenMatrix.cpp)


