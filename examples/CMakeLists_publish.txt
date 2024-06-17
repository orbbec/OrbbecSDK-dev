cmake_minimum_required(VERSION 3.1.15)
project(OrbbecSDK-Samples LANGUAGES CXX C)

IF(UNIX)
    set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")
ENDIF()

# obsensor sdk
set(OrbbecSDK_ROOT_DIR ${CMAKE_SOURCE_DIR}/../SDK)
set(OrbbecSDK_LIBRARY_DIRS ${OrbbecSDK_ROOT_DIR}/lib)
set(OrbbecSDK_INCLUDE_DIR ${OrbbecSDK_ROOT_DIR}/include)
include_directories(${OrbbecSDK_INCLUDE_DIR})
link_directories(${OrbbecSDK_LIBRARY_DIRS})

# find opencv
# set(OpenCV_DIR "your/path/to/opencv") # Alternative you can input cmake parameter from when cmake building
if(NOT DEFINED OpenCV_DIR)
  # Get OpenCV from system environment
  if(DEFINED ENV{OpenCV_INSTALL_PATH})
    set(OpenCV_DIR $ENV{OpenCV_INSTALL_PATH})
  endif()
endif()
find_package(OpenCV REQUIRED)
if(DEFINED OpenCV_LIBS AND DEFINED OpenCV_INCLUDE_DIRS)
  set(OpenCV_FOUND TRUE)
  include_directories(${OpenCV_INCLUDE_DIRS})
else()
  set(OpenCV_FOUND FALSE)
endif()

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(UNIX)
#linux system set sample rpath
    set(CMAKE_SKIP_BUILD_RPATH FALSE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
    set(CMAKE_INSTALL_RPATH "$ORIGIN:${CMAKE_INSTALL_RPATH}:${OrbbecSDK_LIBRARY_DIRS}")
    if(NOT APPLE)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
    endif()
endif()

add_subdirectory(c)
add_subdirectory(cpp)
