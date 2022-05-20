set(nullmax_include_DIR 
    ${CMAKE_CURRENT_SOURCE_DIR}/nullmax/include/bird-view
    ${CMAKE_CURRENT_SOURCE_DIR}/nullmax/include/ipc
    ${CMAKE_CURRENT_SOURCE_DIR}/nullmax/include/lane_perception
    ${CMAKE_CURRENT_SOURCE_DIR}/nullmax/include/map-tracking
    ${CMAKE_CURRENT_SOURCE_DIR}/nullmax/include/calibration
    ${CMAKE_CURRENT_SOURCE_DIR}/nullmax/include
)
if(VIBRANTE AND NOT QNX)
    set(nullmax_lib_DIR "${CMAKE_CURRENT_SOURCE_DIR}/nullmax/lib/${SDK_ARCH_DIR}/${PLATFORM_DIR}" CACHE PATH '' FORCE)
else()
    set(nullmax_lib_DIR "${CMAKE_CURRENT_SOURCE_DIR}/nullmax/lib/${SDK_ARCH_DIR}" CACHE PATH '' FORCE)
endif()

mark_as_advanced(nullmax_include_DIR nullmax_lib_DIR)
