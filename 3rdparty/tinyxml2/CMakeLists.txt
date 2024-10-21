cmake_minimum_required(VERSION 3.5)

add_library(tinyxml2 STATIC src/tinyxml2.cpp)
target_include_directories(tinyxml2 INTERFACE ${CMAKE_CURRENT_LIST_DIR}/src)

add_library(tinyxml2::tinyxml2 ALIAS tinyxml2)
ob_source_group(tinyxml2::tinyxml2)

set_target_properties(tinyxml2 PROPERTIES FOLDER "dependencies")
