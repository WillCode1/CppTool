cmake_minimum_required(VERSION 2.8)
project(calibrationAndUndistort)

add_executable(calibrationAndUndistort calibration.cpp undistort.cpp main.cpp)
# 链接OpenCV库
target_link_libraries(calibrationAndUndistort ${OpenCV_LIBS})
