cmake_minimum_required(VERSION 3.1.15)

set(OB_TARGET_DEVICE device)
add_library(${OB_TARGET_DEVICE} STATIC)
set_target_properties(${OB_TARGET_DEVICE} PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

target_link_libraries(${OB_TARGET_DEVICE} PUBLIC ob::shared ob::core ob::platform)
target_include_directories(${OB_TARGET_DEVICE} PUBLIC ${OB_PUBLIC_HEADERS_DIR}  ${OB_INTERFACE_DIR} ${CMAKE_CURRENT_LIST_DIR} )

file(GLOB SOURCE_FILES ${CMAKE_CURRENT_LIST_DIR}/*.cpp)
file(GLOB HEADER_FILES ${CMAKE_CURRENT_LIST_DIR}/*.hpp)
target_sources(${OB_TARGET_DEVICE} PRIVATE ${SOURCE_FILES} ${HEADER_FILES})

add_subdirectory(enumerator)
add_subdirectory(property)
add_subdirectory(sensor)

add_library(ob::device ALIAS ${OB_TARGET_DEVICE})
ob_source_group(ob::device)
set_target_properties(${OB_TARGET_DEVICE} PROPERTIES FOLDER "modules")
