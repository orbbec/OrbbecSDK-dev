
cmake_minimum_required(VERSION 3.5)

target_sources(
    ${OB_TARGET_DEVICE}
    PRIVATE ${CMAKE_CURRENT_LIST_DIR}/VendorPropertyAccessor.cpp
            ${CMAKE_CURRENT_LIST_DIR}/VendorPropertyAccessor.hpp
            ${CMAKE_CURRENT_LIST_DIR}/PropertyServer.cpp
            ${CMAKE_CURRENT_LIST_DIR}/PropertyServer.hpp
            ${CMAKE_CURRENT_LIST_DIR}/CommonPropertyAccessors.cpp
            ${CMAKE_CURRENT_LIST_DIR}/CommonPropertyAccessors.hpp
            ${CMAKE_CURRENT_LIST_DIR}/InternalProperty.hpp
)

if(OB_BUILD_USB_PORT)
    target_sources(
        ${OB_TARGET_DEVICE} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/UvcPropertyAccessor.cpp
                                    ${CMAKE_CURRENT_LIST_DIR}/UvcPropertyAccessor.hpp
    )
endif()