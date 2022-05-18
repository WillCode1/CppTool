cmake_minimum_required(VERSION 3.1.0)

PROJECT(front-camera-extrinsic-calibration C CXX)
set(PROJECT_CMAKE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_MODULE_PATH "${PROJECT_CMAKE_DIR}" "${CMAKE_MODULE_PATH}")


include(SetBuildType)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(BUILD_SAMPLES "Build samples" ON)
option(BUILD_TESTS "Build tests" OFF)

include(ArchConfiguration)
include(3rdparty)
set(SDK_BINARY_DIR ${CMAKE_BINARY_DIR})
#set(CMAKE_CXX_FLAGS_RELEASE "-O2 -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(BUILD_SHARED_LIBS)
  set(LIB_TYPE SHARED)
else()
  set(LIB_TYPE STATIC)
  add_definitions(-DNM_STATIC_LIB)
endif()

if(BUILD_TESTS)
  # TODO not read for gtest
  set(GTEST_LIBS gtest)
endif()

set(CMAKE_DEBUG_POSTFIX "d")
set(CMAKE_RELEASE_POSTFIX "")
set(TARGET_LIB_CALIB nullmax_frontCameraExtrinsicCalib)
set(TARGET_LIB_ADJUST nullmax_frontCameraExtrinsicAdjustment)
if(BUILD_SHARED_LIBS)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
      set(ABSOLUTE_TARGET_LIB_CALIB ${CMAKE_BINARY_DIR}/src/${CMAKE_SHARED_LIBRARY_PREFIX}${TARGET_LIB_CALIB}${CMAKE_DEBUG_POSTFIX}${CMAKE_SHARED_LIBRARY_SUFFIX})
      set(ABSOLUTE_TARGET_LIB_ADJUST ${CMAKE_BINARY_DIR}/src/${CMAKE_SHARED_LIBRARY_PREFIX}${TARGET_LIB_ADJUST}${CMAKE_DEBUG_POSTFIX}${CMAKE_SHARED_LIBRARY_SUFFIX})
    elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
      set(ABSOLUTE_TARGET_LIB_CALIB ${CMAKE_BINARY_DIR}/src/${CMAKE_SHARED_LIBRARY_PREFIX}${TARGET_LIB_CALIB}${CMAKE_RELEASE_POSTFIX}${CMAKE_SHARED_LIBRARY_SUFFIX})
      set(ABSOLUTE_TARGET_LIB_ADJUST ${CMAKE_BINARY_DIR}/src/${CMAKE_SHARED_LIBRARY_PREFIX}${TARGET_LIB_ADJUST}${CMAKE_RELEASE_POSTFIX}${CMAKE_SHARED_LIBRARY_SUFFIX})
    else()
      message(FATAL_ERROR "Not supported build type")
    endif()
else()
  set(ABSOLUTE_TARGET_LIB_CALIB ${TARGET_LIB_CALIB})
  set(ABSOLUTE_TARGET_LIB_ADJUST ${TARGET_LIB_ADJUST})
endif()

include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/include/calibration)
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/include/adjustment)

add_subdirectory( src/calibration)
add_subdirectory( include/calibration )
add_subdirectory( src/adjustment)
add_subdirectory( include/adjustment )

if(BUILD_SAMPLES)
  add_subdirectory(samples)
  # add_subdirectory( samples/calibration )
  # add_subdirectory( samples/adjustment )
endif()

if(BUILD_TESTS)
# TODO: not test now
  #set(BUILD_SHARED_LIBS OFF)
  add_subdirectory( src/calibration/test )
  add_subdirectory( src/adjustment/test )
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Build shared libs: ${BUILD_SHARED_LIBS}")
message(STATUS "Build samples: ${BUILD_SAMPLES}")
message(STATUS "Build tests: ${BUILD_TESTS}")
