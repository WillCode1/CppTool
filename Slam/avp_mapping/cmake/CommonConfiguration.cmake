# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# Debug symbols
#-------------------------------------------------------------------------------
# Enable minimal (level 1) debug info on experimental builds for
# informative stack trace including function names
if(SDK_BUILD_EXPERIMENTAL AND NOT CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g1")
endif()

#-------------------------------------------------------------------------------
# Enable C++11
#-------------------------------------------------------------------------------
if(CMAKE_VERSION VERSION_GREATER 3.1)
    set(CMAKE_CXX_STANDARD 11)
else()
    if(LINUX OR VIBRANTE)
        include(CheckCXXCompilerFlag)
        CHECK_CXX_COMPILER_FLAG(-std=c++11 COMPILER_SUPPORTS_CXX11)
        CHECK_CXX_COMPILER_FLAG(-std=c++0x COMPILER_SUPPORTS_CXX0X)
        if(COMPILER_SUPPORTS_CXX11)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
        elseif(COMPILER_SUPPORTS_CXX0X)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
        else()
            message(ERROR "Compiler ${CMAKE_CXX_COMPILER} has no C++11 support")
        endif()
    endif()
endif()

#-------------------------------------------------------------------------------
# Dependencies
#-------------------------------------------------------------------------------
find_package(Threads REQUIRED)

#-------------------------------------------------------------------------------
# Profiling
#-------------------------------------------------------------------------------
if (CMAKE_BUILD_TYPE MATCHES "Profile")
    add_definitions(-DDW_PROFILING)
    set(DW_PROFILING TRUE)
endif()
