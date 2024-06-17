#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-PostProcessing)

add_executable(PostProcessing PostProcessing.cpp)

set_property(TARGET PostProcessing PROPERTY CXX_STANDARD 11)
target_link_libraries(PostProcessing OrbbecSDK ${OpenCV_LIBS})

