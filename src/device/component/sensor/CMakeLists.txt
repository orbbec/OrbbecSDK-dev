#cmake_minimum_required(VERSION 3.5)

target_sources(${OB_TARGET_DEVICE} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/SensorBase.cpp ${CMAKE_CURRENT_LIST_DIR}/SensorBase.hpp)

add_subdirectory(motion)
add_subdirectory(video)