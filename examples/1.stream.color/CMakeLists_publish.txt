#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-ColorViewer)

add_executable(ColorViewer ColorViewer.cpp)

set_property(TARGET ColorViewer PROPERTY CXX_STANDARD 11)
target_link_libraries(ColorViewer OrbbecSDK ${OpenCV_LIBS})