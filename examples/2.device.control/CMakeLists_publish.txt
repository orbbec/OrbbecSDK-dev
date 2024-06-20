#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-SensorControl)

add_executable(SensorControl SensorControl.cpp)

set_property(TARGET SensorControl PROPERTY CXX_STANDARD 11)
target_link_libraries(SensorControl OrbbecSDK ${OpenCV_LIBS})
