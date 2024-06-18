cmake_minimum_required(VERSION 3.1.15)

set(OB_INTERFACE_DIR "${CMAKE_CURRENT_LIST_DIR}/interface")

# modules
add_subdirectory(shared) # utils, logger, etc.
add_subdirectory(core) # core library
add_subdirectory(filter)
add_subdirectory(platform)
add_subdirectory(device)
add_subdirectory(pipeline)

# config version info
if (MSVC)
    set(ORBBEC_VERSIONINFO_RC "${CMAKE_BINARY_DIR}/VersionInfo.rc")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/VersionInfo.rc.in "${ORBBEC_VERSIONINFO_RC}")
endif ()

# only support shared library for now
add_library(openobsdk SHARED "" ${ORBBEC_VERSIONINFO_RC})
include(GenerateExportHeader)
generate_export_header(openobsdk BASE_NAME OB EXPORT_FILE_NAME ${OB_PUBLIC_HEADERS_DIR}/openobsdk/h/Export.h)

file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_LIST_DIR}/impl/*.cpp ${CMAKE_CURRENT_LIST_DIR}/context/*.cpp)
file(GLOB_RECURSE HEADER_FILES ${CMAKE_CURRENT_LIST_DIR}/impl/*.hpp ${CMAKE_CURRENT_LIST_DIR}/context/*.hpp ${CMAKE_CURRENT_LIST_DIR}/../include/openobsdk/hpp/*.hpp)


target_sources(openobsdk PRIVATE ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(openobsdk PRIVATE ob::shared ob::core ob::filter ob::platform ob::device ob::pipeline)


target_include_directories(openobsdk PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(
        openobsdk PUBLIC "$<BUILD_INTERFACE:${OB_PUBLIC_HEADERS_DIR}>" "$<INSTALL_INTERFACE:include>"
)

add_library(ob::openobsdk ALIAS openobsdk)
ob_source_group(ob::openobsdk)

install(
        TARGETS openobsdk
        EXPORT openobsdkTarget
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
)

install(
        DIRECTORY ${OB_PUBLIC_HEADERS_DIR}
        DESTINATION include
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
)

install(EXPORT openobsdkTarget NAMESPACE ob:: DESTINATION lib/cmake)
