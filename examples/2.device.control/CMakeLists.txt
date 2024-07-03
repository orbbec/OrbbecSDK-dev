#  minimum required cmake version: 3.5 support vs2019

cmake_minimum_required(VERSION 3.5)
project(ob_device_control)

add_executable(${PROJECT_NAME} device_control.cpp)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} ob::openobsdk ob::examples::common )

set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER "Examples")

# ob_install_example_target(${PROJECT_NAME})
# ob_install_example_current_source("/cpp/Sample-${PROJECT_NAME}")