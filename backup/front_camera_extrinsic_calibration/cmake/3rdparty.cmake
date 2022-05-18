#-------------------------------------------------------------------------------
# Dependencies
#-------------------------------------------------------------------------------
set(3rdparty_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty" CACHE PATH '' FORCE)
set(opencv_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/opencv" CACHE PATH '' FORCE)
set(ceres_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/ceres" CACHE PATH '' FORCE)

# Hide settings in default cmake view
mark_as_advanced(3rdparty_DIR opencv_DIR)
