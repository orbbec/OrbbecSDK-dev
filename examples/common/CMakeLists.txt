
cmake_minimum_required(VERSION 3.5)
project(ob_examples_common)

add_library(ob_examples_common STATIC
    ${CMAKE_CURRENT_LIST_DIR}/utils_c.c
    ${CMAKE_CURRENT_LIST_DIR}/utils_c.h
    ${CMAKE_CURRENT_LIST_DIR}/utils.cpp
    ${CMAKE_CURRENT_LIST_DIR}/utils.hpp
)

if(${OpenCV_FOUND})
    target_sources(ob_examples_common PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/utils_opencv.cpp
        ${CMAKE_CURRENT_LIST_DIR}/utils_opencv.hpp
    )
    target_link_libraries(ob_examples_common PUBLIC ${OpenCV_LIBS} ob::openobsdk)
    target_include_directories(ob_examples_common PUBLIC ${OpenCV_INCLUDE_DIRS})
endif()

target_include_directories(ob_examples_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(ob::examples::common ALIAS ob_examples_common)
set_target_properties(ob_examples_common PROPERTIES FOLDER "examples")