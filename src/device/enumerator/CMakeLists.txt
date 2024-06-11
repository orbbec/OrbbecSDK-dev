#  minimum required cmake version: 3.1.15 support vs2019
cmake_minimum_required(VERSION 3.1.15)

if(OB_BUILD_USB_PORT)
    file(GLOB _usb_src_files
        UsbDeviceEnumerator.cpp
        UsbDeviceEnumerator.hpp
    )
    list(APPEND _src_files ${_usb_src_files})
endif()

if(OB_BUILD_NET_PORT)
    file(GLOB _net_src_files
        NetDeviceEnumerator.cpp
        NetDeviceEnumerator.hpp
    )
    list(APPEND _src_files ${_net_src_files})
endif()

foreach(_file IN ITEMS ${_src_files})
    target_sources(${OB_TARGET_DEVICE} PRIVATE "${_file}")
endforeach()