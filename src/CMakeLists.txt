cmake_minimum_required(VERSION 3.1.15)

add_subdirectory(core) # core library
add_subdirectory(utils)
add_subdirectory(platform)
add_subdirectory(device)
add_subdirectory(pipeline)
add_subdirectory(filter)
add_subdirectory(impl)

# config version info
if(MSVC)
    set(ORBBEC_VERSIONINFO_RC "${CMAKE_BINARY_DIR}/VersionInfo.rc")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/VersionInfo.rc.in "${ORBBEC_VERSIONINFO_RC}")
endif()

if(BUILD_SHARED_LIBS)
    add_library(${CMAKE_PROJECT_NAME} SHARED "" ${ORBBEC_VERSIONINFO_RC})
    add_definitions(-DOB_EXPORTS)
else()
    add_library(${CMAKE_PROJECT_NAME} STATIC "" ${ORBBEC_VERSIONINFO_RC})
    add_definitions(-DOB_STATIC)
endif()
