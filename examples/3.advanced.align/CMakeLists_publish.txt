#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-Align)

add_executable(AlignFilterViewer AlignFilterViewer.cpp)

set_property(TARGET AlignFilterViewer PROPERTY CXX_STANDARD 11)
target_link_libraries(AlignFilterViewer OrbbecSDK ${OpenCV_LIBS})