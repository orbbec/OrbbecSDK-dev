cmake_minimum_required(VERSION 3.1.15)

# add the core library with version number
add_library(core STATIC)
set_target_properties(core PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

# add the core library sources
file(GLOB_RECURSE SOURCE_FILES "*.cpp" EXCLUDE unittest)
file(GLOB_RECURSE HEADERS_FILES "*.hpp" EXCLUDE unittest)
target_sources(core PRIVATE ${SOURCE_FILES} ${HEADERS_FILES})

target_link_libraries(core PUBLIC ob::shared)
target_include_directories(core PUBLIC "$<BUILD_INTERFACE:${OB_PUBLIC_HEADERS_DIR}>" "$<INSTALL_INTERFACE:include>"
                                       "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>" "$<INSTALL_INTERFACE:include>")

add_library(ob::core ALIAS core)
ob_source_group(ob::core)
set_target_properties(core PROPERTIES FOLDER "shared")

if(INSTALL_MODULE_CORE)
    ob_install_module(core ${CMAKE_CURRENT_SOURCE_DIR})
    install(FILES ${OB_PUBLIC_HEADERS_DIR}/openobsdk/h/ObTypes.h DESTINATION include/openobsdk/h)
endif()
