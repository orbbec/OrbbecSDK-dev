cmake_minimum_required(VERSION 3.5)

add_subdirectory(enumerator)
add_subdirectory(uvc)
add_subdirectory(hid)
add_subdirectory(vendor)

target_sources(${OB_TARGET_PAL} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/UsbGroup.hpp ${CMAKE_CURRENT_LIST_DIR}/UsbGroup.cpp)
