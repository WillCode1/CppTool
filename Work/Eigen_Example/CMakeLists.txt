# 声明要求的 cmake 最低版本
cmake_minimum_required(VERSION 3.21)

# 声明一个 cmake 工程
project(Eigen_Example)

# 设置编译模式
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

# 添加头文件
#include_directories("/usr/include/eigen3")
include_directories(${EIGEN3_INCLUDE_DIRS})

# 添加一个可执行程序
add_executable(EigenExample main.cpp Pose.cpp SolveMatrix.cpp)
