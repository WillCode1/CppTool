find_package(Pangolin REQUIRED)

set(CMAKE_CXX_STANDARD 14)

add_executable(stereoVision stereoVision.cpp)
target_link_libraries(stereoVision ${OpenCV_LIBS} ${Pangolin_LIBRARIES})