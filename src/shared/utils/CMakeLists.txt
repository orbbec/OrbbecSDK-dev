cmake_minimum_required(VERSION 3.1.15)

# add the core library with version number
add_library(utils STATIC)
set_target_properties(utils PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

# add the core library sources
file(GLOB_RECURSE CORE_SOURCES "*.cpp" EXCLUDE unittest)
file(GLOB_RECURSE CORE_HEADERS "*.hpp" EXCLUDE unittest)
target_sources(utils PRIVATE ${CORE_SOURCES} ${CORE_HEADERS})

target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(ob::utils ALIAS utils)
