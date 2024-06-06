cmake_minimum_required(VERSION 3.1.15)

# add the shared library with version number
add_library(shared STATIC
        config/Config.cpp
        config/Config.hpp)
set_target_properties(shared PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

# add the shared library sources
file(GLOB_RECURSE SOURCE_FILES "*.cpp" EXCLUDE)
file(GLOB_RECURSE HEADERS_FILES "*.hpp" EXCLUDE)
target_sources(shared PRIVATE ${SOURCE_FILES} ${HEADERS_FILES})

add_subdirectory(${OB_3RDPARTY_DIR}/spdlog spdlog)
target_link_libraries(shared PUBLIC spdlog::spdlog)
target_include_directories(shared PUBLIC ${OB_PUBLIC_HEADERS_DIR} ${CMAKE_CURRENT_LIST_DIR})

add_library(ob::shared ALIAS shared)
ob_source_group(ob::shared)
set_target_properties(shared PROPERTIES FOLDER "modules")