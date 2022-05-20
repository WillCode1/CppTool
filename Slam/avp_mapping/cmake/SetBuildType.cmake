# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# This include makes sure that only (Debug|Release) build types are used
# If no build is specified for a single-configuration generator, Release is used
# by default.
#-------------------------------------------------------------------------------
if("${CMAKE_GENERATOR}" MATCHES "Visual Studio")
    #Multi-configuration system, set configuration types
    set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configurations")
else()
    #Single-configuration system, check build type
    if(CMAKE_BUILD_TYPE)
        if(NOT "${CMAKE_BUILD_TYPE}" MATCHES "^(Debug|Release)$")
            message(WARNING "CMAKE_BUILD_TYPE must be one of (Debug|Release). Using Release as default.")
            set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type. Options: Debug,Release" FORCE)
        endif()
    else()
        message(WARNING "CMAKE_BUILD_TYPE not defined. Using Release as default.")
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type. Options: Debug,Release")
    endif()
endif()

#This flag "CMAKE_BUILD_TARGET" makes sure that only (Executable|Library) build targets are used
#Executable is used by default
if(CMAKE_BUILD_TARGET)
    if(NOT "${CMAKE_BUILD_TARGET}" MATCHES "^(Executable|Library)$")
        message(WARNING "CMAKE_BUILD_TARGET must be one of (Executable|Library). Using Executable as default.")
        set(CMAKE_BUILD_TARGET "Executable" CACHE STRING "Build target. Options: Executable,Library" FORCE)
    endif()
else()
    message(WARNING "CMAKE_BUILD_TARGET not defined. Using Executable as default.")
    set(CMAKE_BUILD_TARGET "Executable" CACHE STRING "Build target. Options: Executable,Library")
endif()
