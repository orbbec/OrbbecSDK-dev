
cmake_minimum_required(VERSION 3.1.15)

target_sources(
    ${OB_TARGET_DEVICE}
    PRIVATE ${CMAKE_CURRENT_LIST_DIR}/VendorPropertyPort.cpp
            ${CMAKE_CURRENT_LIST_DIR}/VendorPropertyPort.hpp
            ${CMAKE_CURRENT_LIST_DIR}/PropertyAccessor.cpp
            ${CMAKE_CURRENT_LIST_DIR}/PropertyAccessor.hpp
            ${CMAKE_CURRENT_LIST_DIR}/InternalProperty.hpp
            ${CMAKE_CURRENT_LIST_DIR}/FilterPropertyPort.hpp
            ${CMAKE_CURRENT_LIST_DIR}/FilterPropertyPort.cpp
)

if(OB_BUILD_USB_PORT)
    target_sources(
        ${OB_TARGET_DEVICE} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/UvcPropertyPort.cpp
                                    ${CMAKE_CURRENT_LIST_DIR}/UvcPropertyPort.hpp
    )
endif()