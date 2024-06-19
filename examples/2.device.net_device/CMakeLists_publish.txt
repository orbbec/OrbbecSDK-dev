#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-NetDevice)

add_executable(NetDevice NetDevice.cpp)

set_property(TARGET NetDevice PROPERTY CXX_STANDARD 11)
target_link_libraries(NetDevice OrbbecSDK ${OpenCV_LIBS})

