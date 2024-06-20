cmake_minimum_required(VERSION 3.1.15)

set(OB_TARGET_DEVICE device)
add_library(${OB_TARGET_DEVICE} STATIC)
set_target_properties(${OB_TARGET_DEVICE} PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

file(GLOB SOURCE_FILES ${CMAKE_CURRENT_LIST_DIR}/*.cpp)
file(GLOB HEADER_FILES ${CMAKE_CURRENT_LIST_DIR}/*.hpp)
target_sources(${OB_TARGET_DEVICE} PRIVATE ${SOURCE_FILES} ${HEADER_FILES})

target_link_libraries(${OB_TARGET_DEVICE} PUBLIC ob::shared ob::core ob::platform ob::filter)
target_include_directories(${OB_TARGET_DEVICE} PUBLIC ${OB_PUBLIC_HEADERS_DIR}  ${OB_INTERFACE_DIR} ${CMAKE_CURRENT_LIST_DIR} )

# common:
add_subdirectory(context)
add_subdirectory(devicemanager)
add_subdirectory(component)

# devices support:
add_subdirectory(gemini330) # Gemini 330 series

add_library(ob::device ALIAS ${OB_TARGET_DEVICE})
ob_source_group(ob::device)
set_target_properties(${OB_TARGET_DEVICE} PROPERTIES FOLDER "modules")
