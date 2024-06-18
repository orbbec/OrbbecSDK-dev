#  minimum required cmake version: 3.1.15 support vs2019
cmake_minimum_required(VERSION 3.1.15)

target_sources(${OB_TARGET_DEVICE} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/VideoSensor.cpp ${CMAKE_CURRENT_LIST_DIR}/VideoSensor.hpp)
