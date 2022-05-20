# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# Samples Installation configuration
#-------------------------------------------------------------------------------
set(SDK_SAMPLE_DESTINATION  "bin")
set(SDK_LIBRARY_DESTINATION "lib")
set(SDK_ARCHIVE_DESTINATION "lib")

function(sdk_add_sample SAMPLES)
    install(TARGETS ${SAMPLES}
        COMPONENT samples
        RUNTIME DESTINATION ${SDK_SAMPLE_DESTINATION}
        LIBRARY DESTINATION ${SDK_LIBRARY_DESTINATION}
        ARCHIVE DESTINATION ${SDK_ARCHIVE_DESTINATION}
    )
endfunction()

function(sdk_install_shared SUBFOLDER SHARES COMPONENT)
    install(FILES ${SHARES}
        COMPONENT ${COMPONENT}
        DESTINATION "${SDK_SAMPLE_DESTINATION}"
    )
endfunction(sdk_install_shared)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${SDK_BINARY_DIR}/install" CACHE PATH "Default install path" FORCE)
endif()
message(STATUS "Driveworks Samples install dir: ${CMAKE_INSTALL_PREFIX}")
