
cmake_minimum_required(VERSION 3.1.15)
project(ob_examples_common)

add_library(ob_examples_common STATIC
    ${CMAKE_CURRENT_LIST_DIR}/utils_c.c
    ${CMAKE_CURRENT_LIST_DIR}/utils_c.h
    ${CMAKE_CURRENT_LIST_DIR}/utils.cpp
    ${CMAKE_CURRENT_LIST_DIR}/utils.hpp
    ${CMAKE_CURRENT_LIST_DIR}/utils_opencv.hpp
)

target_include_directories(ob_examples_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(ob::examples::common ALIAS ob_examples_common)
set_target_properties(ob_examples_common PROPERTIES FOLDER "Examples")