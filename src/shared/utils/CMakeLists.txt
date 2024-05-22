cmake_minimum_required(VERSION 3.1.15)

# add the core library with version number
add_library(utils STATIC)
set_target_properties(utils PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

# add the core library sources
file(GLOB_RECURSE SOURCE_FILES "*.cpp" EXCLUDE unittest)
file(GLOB_RECURSE HEADERS_FILES "*.hpp" EXCLUDE unittest)
target_sources(utils PRIVATE ${SOURCE_FILES} ${HEADERS_FILES})

target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(ob::utils ALIAS utils)
