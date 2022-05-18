set(lib_name nullmax_frontCameraExtrinsicAdjustment)

include_directories(${opencv_DIR}/include)
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

set(EIGEN3_INCLUDE_DIR ${eigen3_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

file(GLOB lib_src "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/utils/*.cpp")  
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/utils)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

add_library(${lib_name} ${LIB_TYPE} ${lib_src})
if(UNIX)
    if(CMAKE_COMPILER_IS_GNUCC)
         set_target_properties(${lib_name} PROPERTIES CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fvisibility=hidden")
         set_target_properties(${lib_name} PROPERTIES 
             CXX_STANDARD 11
             CXX_STANDARD_REQUIRED YES
             CXX_EXTENSIONS NO
             )
    endif(CMAKE_COMPILER_IS_GNUCC)
endif(UNIX)
target_link_libraries(${lib_name}  ${OpenCV_LIBS} )

set_target_properties(${lib_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/)
set_target_properties(${lib_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/)
install(TARGETS ${lib_name} RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION bin)
