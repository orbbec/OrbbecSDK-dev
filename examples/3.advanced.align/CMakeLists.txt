#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(ob_align)

add_executable(ob_align align.cpp)

set_property(TARGET ob_align PROPERTY CXX_STANDARD 11)
target_link_libraries(ob_align ob::openobsdk ob::examples::common opencv::opencv)
target_include_directories(ob_align PUBLIC ${CMAKE_SOURCE_DIR}/3rdparty/OpenCV430/lib/Windows/include/)

IF(USE_PROJECT_FOLDERS)
    set_target_properties (ob_align PROPERTIES FOLDER "Samples")
ENDIF()

# ob_install_example_target(ob_align)
# ob_install_example_current_source("/cpp/Sample-ob_align")