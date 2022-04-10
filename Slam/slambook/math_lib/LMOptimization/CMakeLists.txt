cmake_minimum_required(VERSION 2.8)
project(LMOptimization)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")

add_executable(LM LevenbergMarquardt.cpp)
target_link_libraries(LM ${OpenCV_LIBS})

add_executable(GN GaussNewton.cpp)
target_link_libraries(GN ${OpenCV_LIBS})
