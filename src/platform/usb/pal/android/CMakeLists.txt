# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.
cmake_minimum_required(VERSION 3.5)

file(GLOB _src_files
    AndroidUsbPal.cpp
    AndroidUsbPal.hpp
)

if(BUILD_USB_PORT)
    file(GLOB _usb_src_files
        AndroidUsbDeviceManager.cpp
        AndroidUsbDeviceManager.hpp
    )
    list(APPEND _src_files ${_usb_src_files})
endif()

foreach(_file IN ITEMS ${_src_files})
    target_sources(${OB_TARGET_PAL} PRIVATE "${_file}")
endforeach()

