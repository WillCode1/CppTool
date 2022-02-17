# 声明要求的 cmake 最低版本
cmake_minimum_required(VERSION 3.21)

# 声明一个 cmake 工程
project(Eigen_Tool)

# 设置编译模式
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

# 添加头文件
#include_directories("/usr/include/eigen3")
include_directories(${EIGEN3_INCLUDE_DIRS})

# 添加一个可执行程序
add_executable(EigenTest main.cpp Pose.cpp Common.cpp MatrixAndVector.cpp)

# 添加hello库
#add_library(hello libHelloSLAM.cpp)
# 共享库
#add_library(hello_shared SHARED libHelloSLAM.cpp)

# 添加可执行程序调用hello库中函数
#add_executable(useHello useHello.cpp)
# 将库文件链接到可执行程序上
#target_link_libraries(useHello hello_shared)
