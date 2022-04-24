# Copyright (c) 2019, NULLMAX CORPORATION.  All rights reserved.

if(NOT DEFINED TARGET_CPU)
    set(TDA4 FALSE)
else()
    set(TDA4 TRUE)
    set( ARCH_DIR A72)
endif()

if(NOT TDA4)
    cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)
else()
    cmake_minimum_required(VERSION 3.17.5)
endif()
#-------------------------------------------------------------------------------
# Root project
#-------------------------------------------------------------------------------
project(avp_mapping)
#-------------------------------------------------------------------------------
# Set basic configurations
#-------------------------------------------------------------------------------
set(CMAKE_MODULE_PATH  ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(SDK_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(SDK_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(CMAKE_VERBOSE_MAKEFILE off)

#-------------------------------------------------------------------------------
# System packages
#-------------------------------------------------------------------------------
if(NOT TDA4)
    find_package(CUDA REQUIRED)
endif()
find_package(Threads REQUIRED)

#-------------------------------------------------------------------------------
# Basic configuration
#-------------------------------------------------------------------------------
if(NOT TDA4)
    include(ArchConfiguration)
endif()
include(CommonConfiguration)
if(NOT TDA4)
    include(Configuration)
endif()
include(InstallConfiguration)
set(SDK_BINARY_DIR ${CMAKE_BINARY_DIR})
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -fPIC")
#-------------------------------------------------------------------------------
# Driveworks SDK
#-------------------------------------------------------------------------------

if(NOT TDA4)
    set(COMMON_LIBRARIES ${CUDA_LIBRARIES} ${CUDA_cublas_LIBRARY})
endif()

if(VIBRANTE_V5L)
  list(APPEND COMMON_LIBRARIES  ${CUDA_npp_LIBRARY} ${CUDA_cufft_LIBRARY})
endif()

if(LINUX)
    list(APPEND COMMON_LIBRARIES ${CMAKE_THREAD_LIBS_INIT} dl rt)
elseif(VIBRANTE)
    list(APPEND COMMON_LIBRARIES ${vibrante_LIBRARIES})
    if(NOT QNX)
        list(APPEND COMMON_LIBRARIES nvmedia ${EGL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
    endif()
endif()

#-------------------------------------------------------------------------------
# Cuda dependencies
#-------------------------------------------------------------------------------
# Create a target to update cuda dependencies
if(NOT TDA4)
    add_cuda_dependencies_target(update_cuda_deps ON)
    set_property(TARGET update_cuda_deps PROPERTY FOLDER "Support")
endif()

############### default built type ####################
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

MESSAGE("---------------------------------------")
MESSAGE(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(NOT TDA4)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/${ARCH_DIR}/${PLATFORM_DIR})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/${ARCH_DIR}/${PLATFORM_DIR})
else()
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/bin/${TARGET_CPU}/${TARGET_BUILD})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/lib/${TARGET_CPU}/${TARGET_BUILD})
endif()

##################### OPTIONS ############################

OPTION(ENABLE_VIEWER "Enable Viewer(OpenCV Pangolin)" OFF)

if(NOT LINUX)
    message("-- Pangolin and OpenCV Viewer Not Support On ${ARCH_DIR}  ${PLATFORM_DIR}")
    set(ENABLE_VIEWER OFF)
endif()

if(ENABLE_VIEWER)
    message("-- Use Pangolin and OpenCV for Visualization ")
    find_package(Pangolin REQUIRED)
    include_directories(${Pangolin_INCLUDE_DIRS})
endif()


CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config/ViewerConfig.h.in
    ${PROJECT_SOURCE_DIR}/include/viewerconfig.h)


################### 3rd-party ##################################

set(3rdparty_dir ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/)


### opencv 
include_directories(${3rdparty_dir}/opencv/include)
link_directories(${3rdparty_dir}/opencv/lib/${ARCH_DIR}/${PLATFORM_DIR})
if(NOT TDA4)
    set(OpenCV_LIBS -lopencv_calib3d -lopencv_features2d -lopencv_imgcodecs -lopencv_imgproc
        -lopencv_highgui -lopencv_core -lopencv_flann  -lopencv_cudawarping -lopencv_cudafilters)
else()
    set(OpenCV_LIBS -lopencv_calib3d -lopencv_features2d -lopencv_imgcodecs -lopencv_imgproc
        -lopencv_highgui -lopencv_core -lopencv_flann -lopencv_videoio)
endif()

# set third-party library path 
set(OpenGV_INCLUDE_DIR ${3rdparty_dir}/opengv/include)
set(DBOW_INCLUDE_DIR  ${3rdparty_dir}/dbow2/include)
set(OPENGV_INCLUDE_DIR ${3rdparty_dir}/opengv/include)
set(EIGEN3_INCLUDE_DIR  ${3rdparty_dir}/eigen3/)
set(ADDITIONAL_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}/unsupported)
set(G2O_INC ${3rdparty_dir}/g2o/include)
set(CUDA_LIBS ${COMMON_LIBRARIES} )
set(NMLOG_INC ${3rdparty_dir}/nmlog/include)

if(VIBRANTE)
  list(APPEND CUDA_LIBS  -lz)
endif()

set(DEPEND_LIBS
    ${OpenCV_LIBS}
    g2o
    opengv
    DBoW2
    nmlog
    ${CUDA_LIBS}
    )

if(TDA4)
    list(APPEND DEPEND_LIBS tilog)

    find_library(PNG_LIB png HINTS $ENV{PSDKRA_PATH}/targetfs/usr/lib)
    find_library(JPEG_LIB jpeg HINTS $ENV{PSDKRA_PATH}/targetfs/usr/lib)
    find_library(Z_LIB z HINTS $ENV{PSDKRA_PATH}/targetfs/usr/lib)
    list(APPEND DEPEND_LIBS ${PNG_LIB} ${JPEG_LIB} ${Z_LIB})
endif()
####################### link directories #########################

link_directories(
     ${3rdparty_dir}/g2o/lib/${ARCH_DIR}/${PLATFORM_DIR}
     ${3rdparty_dir}/dbow2/lib/${ARCH_DIR}/${PLATFORM_DIR}
     ${3rdparty_dir}/opengv/lib/${ARCH_DIR}/${PLATFORM_DIR}
     ${3rdparty_dir}/opengv/lib/${ARCH_DIR}/${PLATFORM_DIR}
     ${3rdparty_dir}/nmlog/lib/${ARCH_DIR}/${PLATFORM_DIR}
     ${3rdparty_dir}/tilog/lib/${ARCH_DIR}/${PLATFORM_DIR}
    )

####################### include directories  ######################
include_directories(
    ${3rdparty_dir}
    ${OPENGV_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenGV_INCLUDE_DIR}
    ${DBOW_INCLUDE_DIR}
    ${G2O_INC}
    ${NMLOG_INC}
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/include/interface/
    ${PROJECT_SOURCE_DIR}/include/avp_mapping/
    ${PROJECT_SOURCE_DIR}/include/avp_mapping/g2o/
    ${PROJECT_SOURCE_DIR}/include/feature_mapping/
    ${PROJECT_SOURCE_DIR}/include/viewer/
)


add_subdirectory(src)
add_subdirectory(demo)
