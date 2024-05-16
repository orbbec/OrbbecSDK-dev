message(STATUS "Setting Linux configurations")

set(CMAKE_C_FLAGS
    "${CMAKE_C_FLAGS}   -Wl,-Bsymbolic -fPIC -pedantic -D_DEFAULT_SOURCE -fpermissive -fvisibility=hidden"
)
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS}  -Wl,-Bsymbolic -fPIC -pedantic -Wno-missing-field-initializers -fpermissive -fvisibility=hidden"
)
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS}  -Wno-switch -Wno-multichar -Wsequence-point -Wformat -Wformat-security"
)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")

# string(REPLACE "-O3" "-O0" CMAKE_CXX_FLAGS_RELEASE
# "${CMAKE_CXX_FLAGS_RELEASE}")
set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)
execute_process(
  COMMAND uname -r
  OUTPUT_VARIABLE UNAME_RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
message(-- " Kernel version: " ${UNAME_RESULT})
string(REGEX MATCH "[0-9]+.[0-9]+" LINUX_KERNEL_VERSION ${UNAME_RESULT})
message(STATUS "linux version ${LINUX_KERNEL_VERSION}")

set(OB_CURRENT_OS "linux")
set(BUILD_LINUX_ARM OFF)

if(BUILD_LINUX_ARM64 AND BUILD_LINUX_ARM32)
  message(SEND_ERROR "Obsensor: can not choose two platform at the same time!!")
endif(BUILD_LINUX_ARM64 AND BUILD_LINUX_ARM32)

execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpmachine
                OUTPUT_VARIABLE MACHINE)

if(BUILD_LINUX_ARM64)
  if(${MACHINE} MATCHES "aarch64-linux-gnu")
    add_definitions(-DOS_ARM)
    add_definitions(-DOS_ARM64)
    set(CMAKE_C_FLAGS
        "${CMAKE_C_FLAGS}  -Wl,--allow-shlib-undefined -mstrict-align -ftree-vectorize"
    )
    set(CMAKE_CXX_FLAGS
        "${CMAKE_CXX_FLAGS} -Wl,--allow-shlib-undefined -mstrict-align -ftree-vectorize"
    )
    set(OB_CURRENT_OS "linux_arm64")
    set(BUILD_LINUX_ARM32 OFF)
    set(BUILD_LINUX_ARM64 ON)
    set(BUILD_LINUX_ARM ON)
  else()
    message(SEND_ERROR "Obsensor: check aarch64-linux-gnu not found!!!")
  endif(${MACHINE} MATCHES "aarch64-linux-gnu")
endif(BUILD_LINUX_ARM64)

if(BUILD_LINUX_ARM32)
  if(${MACHINE} MATCHES "arm-linux-gnueabihf")
    add_definitions(-DOS_ARM)
    add_definitions(-DOS_ARM32)
    set(CMAKE_C_FLAGS
        "${CMAKE_C_FLAGS}  -Wl,--allow-shlib-undefined -mfpu=neon -mfloat-abi=hard -ftree-vectorize -latomic -std=c99"
    )
    set(CMAKE_CXX_FLAGS
        "${CMAKE_CXX_FLAGS} -Wl,--allow-shlib-undefined -mfpu=neon -mfloat-abi=hard -ftree-vectorize -latomic"
    )
    set(OB_CURRENT_OS "linux_arm32")
    set(BUILD_LINUX_ARM32 ON)
    set(BUILD_LINUX_ARM64 OFF)
    set(BUILD_LINUX_ARM ON)
  else()
    message(SEND_ERROR "Obsensor: check arm-linux-gnueabihf not found!!!")
  endif()
endif(BUILD_LINUX_ARM32)

if(NOT BUILD_LINUX_ARM)
  message("linux set sse3")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   -mssse3")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -mssse3")
endif()

if(BUILD_LINUX_ARM64)
  message("linux set neon")
  add_definitions(-D__NEON__)
endif()

add_definitions(-DOS_LINUX)
set(BUILD_LINUX ON)
set(USE_PROJECT_FOLDERS OFF)

# target rpath settings
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(CMAKE_INSTALL_RPATH "$ORIGIN:${CMAKE_INSTALL_RPATH}")
