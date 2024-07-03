#  minimum required cmake version: 3.5 support vs2019
cmake_minimum_required(VERSION 3.5)

target_sources(${OB_TARGET_DEVICE} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/Protocol.cpp ${CMAKE_CURRENT_LIST_DIR}/Protocol.hpp)
