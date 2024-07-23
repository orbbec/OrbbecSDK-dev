cmake_minimum_required(VERSION 3.5)

set(OB_TARGET_PAL platform)
add_library(${OB_TARGET_PAL} STATIC)
set_target_properties(${OB_TARGET_PAL} PROPERTIES VERSION ${PROJECT_VERSION})

target_link_libraries(${OB_TARGET_PAL} PUBLIC ob::shared ob::core)
target_include_directories(${OB_TARGET_PAL} PUBLIC
    ${OB_PUBLIC_HEADERS_DIR}
    ${CMAKE_CURRENT_LIST_DIR}
)

add_library(ob::platform ALIAS ${OB_TARGET_PAL})
ob_source_group(ob::platform)

set_target_properties(${OB_TARGET_PAL} PROPERTIES FOLDER "modules")

add_subdirectory(pal)
if (OB_BUILD_USB_PORT)
  add_subdirectory(usb)
  add_subdirectory(${OB_3RDPARTY_DIR}/libusb libusb)
  target_link_libraries(${OB_TARGET_PAL} PUBLIC libusb::libusb)

if(NOT OB_BUILD_WIN32)
  add_subdirectory(${OB_3RDPARTY_DIR}/libuvc libuvc)
  target_link_libraries(${OB_TARGET_PAL} PUBLIC libuvc::libuvc)
endif()

endif ()

if (OB_BUILD_NET_PORT)
  add_subdirectory(ethernet)
  add_subdirectory(${OB_3RDPARTY_DIR}/live555 live555)
  target_link_libraries(${OB_TARGET_PAL} PUBLIC live555::live555)
endif ()