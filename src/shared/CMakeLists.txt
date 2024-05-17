cmake_minimum_required(VERSION 3.1.15)

cmake_minimum_required(VERSION 3.1.15)

# add the shared library with version number
add_library(shared STATIC)
set_target_properties(shared PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

# add the shared library sources
file(GLOB_RECURSE CORE_SOURCES "*.cpp" EXCLUDE )
file(GLOB_RECURSE CORE_HEADERS "*.hpp" EXCLUDE)
target_sources(shared PRIVATE ${CORE_SOURCES} ${CORE_HEADERS})

target_link_libraries(shared PUBLIC spdlog::spdlog)
target_include_directories(shared PUBLIC ${OB_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

add_library(ob::shared ALIAS shared)
ob_source_group(ob::shared)
set_target_properties(shared PROPERTIES FOLDER "shared")
