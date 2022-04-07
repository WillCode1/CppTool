cmake_minimum_required(VERSION 3.21)
project(myslam)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

include_directories(
        common
#        ${catkin_INCLUDE_DIRS}
        "/usr/include/eigen3/"
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
)

add_executable(myslam main.cpp
        laser/LoopClosure.cpp
        laser/CloudDeskew.cpp
        imu/ImuPreintegration.cpp)
target_link_libraries(myslam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)
