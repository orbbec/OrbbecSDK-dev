#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-DepthViewer)

add_executable(DepthViewer DepthViewer.cpp)

set_property(TARGET DepthViewer PROPERTY CXX_STANDARD 11)
target_link_libraries(DepthViewer OrbbecSDK ${OpenCV_LIBS})

