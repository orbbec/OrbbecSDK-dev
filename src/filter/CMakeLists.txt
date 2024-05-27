cmake_minimum_required(VERSION 3.1.15)

# add the filter library with version number
add_library(filter STATIC)
set_target_properties(filter PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

# add the filter library sources
file(GLOB_RECURSE SOURCE_FILES "*.cpp" EXCLUDE unittest)
file(GLOB_RECURSE HEADERS_FILES "*.hpp" EXCLUDE unittest)
target_sources(filter PRIVATE ${SOURCE_FILES} ${HEADERS_FILES})

target_link_libraries(filter PUBLIC ob::shared ob::core)
target_include_directories(filter PUBLIC ${OB_PUBLIC_HEADERS_DIR} ${CMAKE_CURRENT_LIST_DIR})

add_library(ob::filter ALIAS filter)
ob_source_group(ob::filter)

set_target_properties(filter PROPERTIES FOLDER "modules")