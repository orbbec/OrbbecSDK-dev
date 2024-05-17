cmake_minimum_required(VERSION 3.1.15)

file(GLOB subdirectories RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*test*/")

foreach(subdir ${subdirectories})
    add_subdirectory(${subdir})
    message(STATUS "Added subdirectory: ${subdir}")
endforeach()
