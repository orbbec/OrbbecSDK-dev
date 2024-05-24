cmake_minimum_required(VERSION 3.1.15)

cmake_minimum_required(VERSION 3.1.15)

# add the shared library with version number
add_library(shared STATIC)
set_target_properties(shared PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

# add the shared library sources
file(GLOB_RECURSE SOURCE_FILES "*.cpp" EXCLUDE)
file(GLOB_RECURSE HEADERS_FILES "*.hpp" EXCLUDE)
target_sources(shared PRIVATE ${SOURCE_FILES} ${HEADERS_FILES})

add_subdirectory(${OB_3RDPARTY_DIR}/spdlog spdlog)
target_link_libraries(shared PUBLIC spdlog::spdlog)
target_include_directories(shared PUBLIC "$<BUILD_INTERFACE:${OB_PUBLIC_HEADERS_DIR}>" "$<INSTALL_INTERFACE:include>"
                                         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>" "$<INSTALL_INTERFACE:include>")

add_library(ob::shared ALIAS shared)
ob_source_group(ob::shared)
set_target_properties(shared PROPERTIES FOLDER "shared")