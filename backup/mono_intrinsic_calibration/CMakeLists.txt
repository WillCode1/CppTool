cmake_minimum_required(VERSION 3.1.0)
add_definitions(-Wall)
project(mono_intrinsic_calibration)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(PERCEPTION_ROOT ${PROJECT_SOURCE_DIR}/../../)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories(
        ${PERCEPTION_ROOT}/3rdparty/jsoncpp
        ${PERCEPTION_ROOT}/3rdparty/nmlog/include
        ${PERCEPTION_ROOT}/3rdparty/opencv/include
        ${PERCEPTION_ROOT}/3rdparty/ceres/include
        ${PERCEPTION_ROOT}/3rdparty/eigen3
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src
        )

add_library(mono_intrinsic_lib SHARED
   src/mono_intrinsic.cpp
   src/model_base.cpp
   src/pinhole.cpp
   src/fisheye.cpp
   src/ocam.cpp
   src/ocam_algrithom.cpp
)

set(OPENCV_LIB_PATH ${PERCEPTION_ROOT}/3rdparty/opencv/lib/x86_64-linux/)
set(OpenCV_LIBS ${OPENCV_LIB_PATH}/libopencv_cudaarithm.so
            ${OPENCV_LIB_PATH}/libopencv_cudawarping.so ${OPENCV_LIB_PATH}/libopencv_cudacodec.so
            ${OPENCV_LIB_PATH}/libopencv_cudaimgproc.so ${OPENCV_LIB_PATH}/libopencv_cudafilters.so
            ${OPENCV_LIB_PATH}/libopencv_calib3d.so ${OPENCV_LIB_PATH}/libopencv_features2d.so
            ${OPENCV_LIB_PATH}/libopencv_imgcodecs.so ${OPENCV_LIB_PATH}/libopencv_imgproc.so
            ${OPENCV_LIB_PATH}/libopencv_highgui.so ${OPENCV_LIB_PATH}/libopencv_core.so
            ${OPENCV_LIB_PATH}/libopencv_flann.so)
set(Ceres_LIB_PATH ${PERCEPTION_ROOT}/3rdparty/ceres/lib/x86_64-linux/)    
set(CERES_LIBRARIES ${Ceres_LIB_PATH}/libceres.so
            ${Ceres_LIB_PATH}/libceres.so.1
            ${Ceres_LIB_PATH}/libceres.so.1.14.0
            ${Ceres_LIB_PATH}/libglog.so)
set(Jsoncpp_LIB ${PERCEPTION_ROOT}/3rdparty/jsoncpp/lib/x86_64-linux/libjsoncpp.a)

target_link_libraries(mono_intrinsic_lib ${OpenCV_LIBS} ${CERES_LIBRARIES} ${Jsoncpp_LIB})

add_subdirectory(${PROJECT_SOURCE_DIR}/sample)
