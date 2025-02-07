# Copyright (c) Orbbec Inc. All Rights Reserved.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 3.5)

if(NOT WIN32)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-copy")

project(ob_pcl)

find_package(PCL REQUIRED)

if (PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    link_directories(${PCL_LIBRARY_DIRS})
endif()
list(APPEND DEPENDENCIES ${PCL_LIBRARIES})

add_executable(ob_pcl pcl.cpp)

set_property(TARGET ob_pcl PROPERTY CXX_STANDARD 11)
target_link_libraries(ob_pcl ob::OrbbecSDK ob::examples::utils ${DEPENDENCIES})

set_target_properties(ob_pcl PROPERTIES FOLDER "examples")

foreach(flag_var
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO
        CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
        CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO)
    if(${flag_var} MATCHES "/MT")
        string(REGEX REPLACE "/MT" "/MD" ${flag_var} "${${flag_var}}")
    endif(${flag_var} MATCHES "/MT")
endforeach(flag_var)

install(TARGETS ob_pcl RUNTIME DESTINATION bin)