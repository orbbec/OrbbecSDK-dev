#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-HdrMerge)

add_executable(HdrMerge HdrMerge.cpp)

set_property(TARGET HdrMerge PROPERTY CXX_STANDARD 11)
target_link_libraries(HdrMerge OrbbecSDK ${OpenCV_LIBS})