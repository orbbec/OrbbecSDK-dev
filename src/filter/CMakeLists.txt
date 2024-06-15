cmake_minimum_required(VERSION 3.1.15)

# add the filter library with version number
add_library(filter STATIC)
set_target_properties(filter PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

# add the filter library sources
file(GLOB_RECURSE SOURCE_FILES "*.cpp" EXCLUDE unittest)
file(GLOB_RECURSE HEADERS_FILES "*.hpp" EXCLUDE unittest)
target_sources(filter PRIVATE ${SOURCE_FILES} ${HEADERS_FILES})

target_link_libraries(filter PUBLIC ob::shared ob::core)
message("OB_PUBLIC_HEADERS_DIR:${OB_PUBLIC_HEADERS_DIR}")
message("CMAKE_CURRENT_LIST_DIR:${CMAKE_CURRENT_LIST_DIR}")
target_include_directories(filter PUBLIC ${OB_PUBLIC_HEADERS_DIR} ${CMAKE_CURRENT_LIST_DIR})

add_subdirectory(${OB_3RDPARTY_DIR}/dylib dylib)
target_link_libraries(filter PUBLIC dylib::dylib)

add_subdirectory(${OB_3RDPARTY_DIR}/libjpeg libjpeg)
target_link_libraries(filter PUBLIC libjpeg::libjpeg)

add_subdirectory(${OB_3RDPARTY_DIR}/libyuv libyuv)
target_link_libraries(filter PUBLIC libyuv::libyuv)

add_library(ob::filter ALIAS filter)
ob_source_group(ob::filter)

set_target_properties(filter PROPERTIES FOLDER "modules")

if(OB_INSTALL_FILTER_DEV_HEADERS)
    install(FILES ${CMAKE_CURRENT_LIST_DIR}/private_filters/PrivFilterTypes.h DESTINATION include/openobsdk/h/)
endif()
