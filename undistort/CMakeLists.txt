cmake_minimum_required(VERSION 3.10)
project(CameraUndistort)

set(CMAKE_CXX_STANDARD 11)

# 寻找 OpenCV
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(undistort_app main.cpp)
target_link_libraries(undistort_app ${OpenCV_LIBS})