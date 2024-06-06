# cmake_minimum_required(VERSION 3.1.15)

# file(GLOB _cpp_files "${CMAKE_CURRENT_LIST_DIR}/*.c")
# file(GLOB _head_files "${CMAKE_CURRENT_LIST_DIR}/*.h")

# foreach(_file IN ITEMS ${_cpp_files})
# target_sources(${OB_LBS_TARGET} PRIVATE "${_file}")
# endforeach()

# foreach(_file IN ITEMS ${_head_files})
# target_sources(${OB_LBS_TARGET} PRIVATE "${_file}")
# endforeach()

# minimum required cmake version: 3.1.15 support vs2019
cmake_minimum_required(VERSION 3.1.15)

file(GLOB _src_files
    common.h
    dynlib.h
    global.c
    global.h
    handle.h
    k4aplugin.h
    k4atypes.h
)
list(APPEND _src_files ${_src_files})

if(WIN32)
    file(GLOB _platform_files
        dynlib_windows.c
    )
    list(APPEND _src_files ${_platform_files})
else()
    file(GLOB _platform_files
        dynlib_linux.c
    )
    list(APPEND _src_files ${_platform_files})
endif()

foreach(_file IN ITEMS ${_src_files})
    target_sources(${OB_LBS_TARGET} PRIVATE "${_file}")
endforeach()
