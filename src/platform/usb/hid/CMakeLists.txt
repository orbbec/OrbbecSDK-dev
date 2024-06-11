#  minimum required cmake version: 3.1.15 support vs2019
cmake_minimum_required(VERSION 3.1.15)

if(OB_BUILD_WIN32)
    file (GLOB _cpp_files "${CMAKE_CURRENT_LIST_DIR}/*.cpp")
    file (GLOB _head_files "${CMAKE_CURRENT_LIST_DIR}/*.hpp")
else()
	file (GLOB _cpp_files
	    "${CMAKE_CURRENT_LIST_DIR}/HidDevicePort.cpp"
	)
	file (GLOB _head_files
	    "${CMAKE_CURRENT_LIST_DIR}/HidDevicePort.hpp"
	)
endif()

foreach(_file IN ITEMS ${_cpp_files})
        target_sources(${OB_TARGET_PAL} PRIVATE "${_file}")
endforeach()

foreach(_file IN ITEMS ${_head_files})
        target_sources(${OB_TARGET_PAL} PRIVATE "${_file}")
endforeach()