cmake_minimum_required(VERSION 3.1.15)

add_subdirectory(shared) # utils, logger, etc.
add_subdirectory(core) # core library
add_subdirectory(platform)
add_subdirectory(device)
add_subdirectory(pipeline)
add_subdirectory(filter)

# config version info
if(MSVC)
    set(ORBBEC_VERSIONINFO_RC "${CMAKE_BINARY_DIR}/VersionInfo.rc")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/VersionInfo.rc.in "${ORBBEC_VERSIONINFO_RC}")
endif()

if(BUILD_SHARED_LIBS)
    add_library(openobsdk SHARED "" ${ORBBEC_VERSIONINFO_RC})
    add_definitions(-DOB_EXPORTS)
else()
    add_library(openobsdk STATIC "" ${ORBBEC_VERSIONINFO_RC})
    add_definitions(-DOB_STATIC)
endif()

file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_LIST_DIR}/impl/*.cpp ${CMAKE_CURRENT_LIST_DIR}/context/*.cpp)
file(GLOB_RECURSE HEADER_FILES ${CMAKE_CURRENT_LIST_DIR}/impl/*.hpp ${CMAKE_CURRENT_LIST_DIR}/context/*.hpp)

target_sources(openobsdk PUBLIC ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(openobsdk PUBLIC ob::shared ob::core)
target_include_directories(openobsdk PUBLIC ${CMAKE_CURRENT_LIST_DIR})

add_library(ob::openobsdk ALIAS openobsdk)
ob_source_group(ob::openobsdk)
