cmake_minimum_required(VERSION 3.1.15)

add_library(dylib INTERFACE)
target_include_directories(dylib INTERFACE ${CMAKE_CURRENT_LIST_DIR}/src/include)
if(UNIX)
    target_link_libraries(dylib INTERFACE dl)
endif()

add_library(dylib::dylib ALIAS dylib)
ob_source_group(dylib::dylib)

set_target_properties(dylib PROPERTIES FOLDER "dependencies")
