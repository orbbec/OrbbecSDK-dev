# minimum required cmake version: 3.1.15 support vs2019
cmake_minimum_required(VERSION 3.1.15)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project( OpenOrbbecSDK VERSION 0.0.1 LANGUAGES CXX C)

# set project options
include(cmake/options.cmake)

# include macros
include(cmake/macros.cmake)

# configure platform-specific settings
if(ANDROID_NDK_TOOLCHAIN_INCLUDED)
    include(cmake/android_config.cmake)
elseif(WIN32)
    include(cmake/windows_config.cmake)
elseif(APPLE)
    include(cmake/macos_config.cmake)
else(UNIX)
    include(cmake/linux_config.cmake)
endif()

# set output directories
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

string(TOLOWER ${CMAKE_BUILD_TYPE} OB_BUILD_TYPE)
set(OB_OUTPUT_DIRECTORY_ROOT ${CMAKE_BINARY_DIR}/${OB_CURRENT_OS}/${OB_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OB_OUTPUT_DIRECTORY_ROOT}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OB_OUTPUT_DIRECTORY_ROOT}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OB_OUTPUT_DIRECTORY_ROOT}/bin)
if(MSVC)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
        "${OB_OUTPUT_DIRECTORY_ROOT}/lib"
        CACHE PATH "Archive directory" FORCE)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/${OB_CURRENT_OS}/debug/bin")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/${OB_CURRENT_OS}/release/bin")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/${OB_CURRENT_OS}/debug/lib")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/${OB_CURRENT_OS}/release/lib")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/${OB_CURRENT_OS}/debug/lib")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/${OB_CURRENT_OS}/release/lib")
endif()

# set include directories
set(OB_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include)
set(OB_UTILS_DIRS ${CMAKE_CURRENT_LIST_DIR}/src/utils)

add_subdirectory(src)
add_subdirectory(3rdparty/spdlog)

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

if(BUILD_DOCS)
    add_subdirectory(docs)
endif()

if(BUILD_TOOLS)
    add_subdirectory(tools)
endif()

print_summary()
