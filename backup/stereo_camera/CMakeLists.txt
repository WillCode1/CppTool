cmake_minimum_required(VERSION 3.1.0)
add_definitions(-Wall)
project(stereo_calibration_demo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(PERCEPTION_ROOT ${PROJECT_SOURCE_DIR}/../..)
set(EXECUTABLE_OUTPUT_PATH ${PERCEPTION_ROOT}/bin/x86_64-linux)

include_directories(
        ${PERCEPTION_ROOT}/3rdparty/jsoncpp
        ${PERCEPTION_ROOT}/3rdparty/nmlog/include
        ${PERCEPTION_ROOT}/3rdparty/opencv/include)
include_directories(./src)


set(exe_name stereo_calib_demo)
add_executable(${exe_name}
src/stereo_calib.cpp
src/utils.cpp
)

target_link_libraries(${exe_name}
        ${PERCEPTION_ROOT}/3rdparty/opencv/lib/x86_64-linux/libopencv_core.so
        ${PERCEPTION_ROOT}/3rdparty/opencv/lib/x86_64-linux/libopencv_calib3d.so
        ${PERCEPTION_ROOT}/3rdparty/opencv/lib/x86_64-linux/libopencv_imgproc.so
        ${PERCEPTION_ROOT}/3rdparty/opencv/lib/x86_64-linux/libopencv_highgui.so
        ${PERCEPTION_ROOT}/3rdparty/opencv/lib/x86_64-linux/libopencv_imgcodecs.so
        ${PERCEPTION_ROOT}/3rdparty/jsoncpp/lib/x86_64-linux/libjsoncpp.a
        ${PERCEPTION_ROOT}/3rdparty/nmlog/lib/x86_64-linux/libnmlog.so)
