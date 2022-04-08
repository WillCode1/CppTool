cmake_minimum_required(VERSION 3.21)
project(mme)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")

find_package(PCL REQUIRED QUIET)

include_directories(
        common
#        ${catkin_INCLUDE_DIRS}
        "/usr/include/eigen3/"
        ${PCL_INCLUDE_DIRS}
)

add_executable(mme main.cpp)
target_link_libraries(mme ${PCL_LIBRARIES})
