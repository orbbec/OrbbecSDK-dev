# minimum required cmake version: 3.1.15 support vs2019
cmake_minimum_required(VERSION 3.1.15)
project(
    OpenOrbbecSDK
    VERSION 0.0.1
    LANGUAGES CXX C)

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

add_subdirectory(src)

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
