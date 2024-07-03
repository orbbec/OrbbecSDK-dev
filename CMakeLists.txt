
cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(OpenOrbbecSDK VERSION 2.0.0 LANGUAGES CXX C)

# load project options
include(cmake/options.cmake)
if(OB_BUILD_USB_PORT)
    add_definitions(-DBUILD_USB_PORT)
endif()
if(OB_BUILD_NET_PORT)
    add_definitions(-DBUILD_NET_PORT)
endif()

# load macros
include(cmake/macros.cmake)

# set compiler flags
include(cmake/compiler_flags.cmake)

# configure platform-specific settings（contains platform-specific compiler flags）
if(ANDROID_NDK_TOOLCHAIN_INCLUDED)
    include(cmake/android_config.cmake)
elseif(WIN32)
    include(cmake/windows_config.cmake)
elseif(APPLE)
    include(cmake/macos_config.cmake)
elseif(UNIX)
    include(cmake/linux_config.cmake)
endif()

# configure global settings: install rules, output directories, global variables, etc.
include(cmake/global_config.cmake)

find_program(CLANG_TIDY_EXE NAMES "clang-tidy" QUIET)

if(CLANG_TIDY_EXE)
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
endif()

# configure global directories variables
set(OB_PUBLIC_HEADERS_DIR ${CMAKE_CURRENT_LIST_DIR}/include/)
set(OB_3RDPARTY_DIR ${CMAKE_CURRENT_LIST_DIR}/3rdparty)
set(OB_PROJECT_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})

add_subdirectory(src)

if(OB_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(OB_BUILD_TESTS)
    add_subdirectory(tests)
endif()

if(OB_BUILD_DOCS)
    add_subdirectory(docs)
endif()

if(OB_BUILD_TOOLS)
    add_subdirectory(tools)
endif()

ob_print_summary()
