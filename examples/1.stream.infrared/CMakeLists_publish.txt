#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-InfraredViewer)

add_executable(InfraredViewer InfraredViewer.cpp)

set_property(TARGET InfraredViewer PROPERTY CXX_STANDARD 11)
target_link_libraries(InfraredViewer OrbbecSDK ${OpenCV_LIBS})
