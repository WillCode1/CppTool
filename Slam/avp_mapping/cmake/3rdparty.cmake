# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# Dependencies
#-------------------------------------------------------------------------------
set(3rdparty_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty" CACHE PATH '' FORCE)
set(nvidia_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/${PLATFORM_DIR}/nvidia" CACHE PATH '' FORCE)

if(VIBRANTE AND NOT QNX)
  set(jsoncpp_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/${PLATFORM_DIR}/jsoncpp" CACHE PATH '' FORCE)
  set(deeplearning_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/${PLATFORM_DIR}/deeplearning" CACHE PATH '' FORCE)
  set(3rdparty_common_lib_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/${PLATFORM_DIR}/lib" CACHE PATH '' FORCE)
  set(opencv_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/${PLATFORM_DIR}/opencv" CACHE PATH '' FORCE)
  set(gtest_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/${PLATFORM_DIR}/gtest" CACHE PATH '' FORCE)
else()
  set(jsoncpp_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/jsoncpp" CACHE PATH '' FORCE)
  set(deeplearning_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/deeplearning" CACHE PATH '' FORCE)
  set(3rdparty_common_lib_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/lib" CACHE PATH '' FORCE)
  set(opencv_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/opencv" CACHE PATH '' FORCE)
  set(gtest_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/gtest" CACHE PATH '' FORCE)
endif()

set(3rdparty_include_DIR 
    ${3rdparty_DIR} 
    ${deeplearning_DIR} 
    ${3rdparty_DIR}/eigen3 
    ${3rdparty_DIR}/protobuf
    ${jsoncpp_DIR}
    ${opencv_DIR}/include
    ${3rdparty_DIR}/gtest/include
    ${3rdparty_DIR}/spdlog/include
    ${nvidia_DIR}/include
    ${3rdparty_DIR}/sipl/include
    )

set(3rdparty_lib_DIR
   ${3rdparty_common_lib_DIR}
   ${opencv_DIR}/lib
   ${jsoncpp_DIR}/lib
   ${nvidia_DIR}/lib
)


if(LINUX)
  set(gles_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/gles" CACHE PATH '' FORCE)
  list(APPEND 3rdparty_include_DIR  ${gles_DIR}/include)
  list(APPEND 3rdparty_lib_DIR ${3rdparty_DIR}/${SDK_ARCH_DIR}/nvidia)
  list(APPEND 3rdparty_lib_DIR /usr/lib/nvidia-${nvidia-driver-version})
endif()

if(NOT QNX)
  if(LINUX)
    set(glfw3_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/glfw-3.1.1" CACHE PATH '' FORCE)
    set(glew_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/glew-1.13.0" CACHE PATH '' FORCE)
  else()
    set(glfw3_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/${PLATFORM_DIR}/glfw-3.1.1" CACHE PATH '' FORCE)
    set(glew_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/${PLATFORM_DIR}/glew-1.13.0" CACHE PATH '' FORCE)  
  endif()
  find_package(glfw3 REQUIRED CONFIG)
endif()

if(WINDOWS OR LINUX)
    find_package(glew REQUIRED CONFIG)
endif()

if(VIBRANTE AND NOT QNX)
  set(vibrante_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/${SDK_ARCH_DIR}/${PLATFORM_DIR}/vibrante" CACHE PATH '' FORCE)
  find_package(vibrante REQUIRED CONFIG)
  find_package(EGL REQUIRED)
  add_definitions(-DDW_USE_NVMEDIA)
  add_definitions(-DDW_USE_EGL)
  set(DW_USE_NVMEDIA ON)
  set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
  set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
endif()

# Hide settings in default cmake view
mark_as_advanced(glfw3_DIR glew_DIR  vibrante_DIR 3rdparty_common_lib_DIR 3rdparty_include_DIR 3rdparty_lib_DIR)
