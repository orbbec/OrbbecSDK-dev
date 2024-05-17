cmake_minimum_required(VERSION 3.1.15)

# add the core library with version number
add_library(core STATIC)
set_target_properties(core PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

# add the core library sources
file(GLOB_RECURSE CORE_SOURCES "*.cpp" EXCLUDE unittest)
file(GLOB_RECURSE CORE_HEADERS "*.hpp" EXCLUDE unittest)
target_sources(core PRIVATE ${CORE_SOURCES} ${CORE_HEADERS})

target_link_libraries(core PUBLIC ob::shared)
target_include_directories(core PUBLIC ${OB_INCLUDE_DIRS} ${OB_INTERFACE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

add_library(ob::core ALIAS core)
ob_source_group(ob::core)
set_target_properties(core PROPERTIES FOLDER "shared")
