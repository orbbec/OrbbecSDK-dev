cmake_minimum_required(VERSION 3.1.15)

add_subdirectory(src)
add_library(dylib::dylib ALIAS dylib)
ob_source_group(dylib::dylib)

set_target_properties(dylib PROPERTIES FOLDER "dependencies")


