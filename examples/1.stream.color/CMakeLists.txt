#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(ob_color)
include_directories(${CMAKE_SOURCE_DIR}/include/openobsdk)
add_executable(ob_color color.cpp)

set_property(TARGET ob_color PROPERTY CXX_STANDARD 11)
target_link_libraries(ob_color OrbbecSDK ${OpenCV_LIBS})

set_target_properties (ob_color PROPERTIES FOLDER "Examples")

# ob_install_example_target(ob_color)
# ob_install_example_current_source("/cpp/1.stream.color")
