cmake_minimum_required(VERSION 3.1.0)

include_directories(${opencv_DIR}/include)
include_directories(${opencv_DIR}/include/opencv)
link_directories(${opencv_DIR}/lib/x86_64-linux)
set(OpenCV_LIBS 
-lopencv_highgui
-lopencv_stitching
-lopencv_shape
-lopencv_imgproc
#-lopencv_ml
-lopencv_videostab
-lopencv_calib3d
-lopencv_photo
#-lopencv_dnn
-lopencv_imgcodecs
-lopencv_objdetect
-lopencv_flann
-lopencv_videoio
-lopencv_core
-lopencv_superres
#-lopencv_video
-lopencv_features2d
)

option (OpenCV_STATIC OFF)
set(INCLUDE_PATH ../include)
include_directories(${INCLUDE_PATH})
set(ABSOLUTE_TARGET_LIB_CALIB frontCameraExtrinsicCalibSample)
set(ABSOLUTE_TARGET_LIB_ADJUST frontCameraExtrinsicAdjustmentSample)

add_library(${ABSOLUTE_TARGET_LIB_CALIB} SHARED IMPORTED)
add_library(${ABSOLUTE_TARGET_LIB_ADJUST} SHARED IMPORTED)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set_property(TARGET ${ABSOLUTE_TARGET_LIB_CALIB} PROPERTY IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/bin/libnullmax_frontCameraExtrinsicCalibd.so)
  set_property(TARGET ${ABSOLUTE_TARGET_LIB_ADJUST} PROPERTY IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/bin/libnullmax_frontCameraExtrinsicAdjustmentd.so)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
  set_property(TARGET ${ABSOLUTE_TARGET_LIB_CALIB} PROPERTY IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/bin/libnullmax_frontCameraExtrinsicCalib.so)
  set_property(TARGET ${ABSOLUTE_TARGET_LIB_ADJUST} PROPERTY IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/bin/libnullmax_frontCameraExtrinsicAdjustment.so)
endif()


#For calibration module
set(exe_name front-camera-extrinsic-calib-sample)
file(GLOB sources *.cpp)
add_executable(${exe_name} ./front_camera_extrinsic_calib_sample.cpp)
add_dependencies(${exe_name} ${TARGET_LIB_CALIB})
target_link_libraries(${exe_name} ${ABSOLUTE_TARGET_LIB_CALIB} ${OpenCV_LIBS})

set_target_properties(${exe_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/)
install(TARGETS ${exe_name} RUNTIME DESTINATION samples/)

#For adjustment module
set(exe_name front-camera-extrinsic-adjustment-sample)
file(GLOB sources *.cpp)
add_executable(front-camera-extrinsic-adjustment-sample ./front_camera_extrinsic_adjustment_sample.cpp)
add_dependencies(${exe_name} ${TARGET_LIB_ADJUST})
target_link_libraries(${exe_name} ${ABSOLUTE_TARGET_LIB_ADJUST} ${OpenCV_LIBS})

set_target_properties(${exe_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/)
install(TARGETS ${exe_name} RUNTIME DESTINATION samples/)
