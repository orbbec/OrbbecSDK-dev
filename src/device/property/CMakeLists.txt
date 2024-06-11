#  minimum required cmake version: 3.1.15 support vs2019
cmake_minimum_required(VERSION 3.1.15)
target_sources(${OB_TARGET_DEVICE} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/InternalProperty.hpp)
if(OB_BUILD_USB_PORT)
    target_sources(${OB_TARGET_DEVICE} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/UvcPropertyPort.cpp ${CMAKE_CURRENT_LIST_DIR}/UvcPropertyPort.hpp)
endif()
