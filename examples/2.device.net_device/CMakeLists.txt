#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(ob_net_device)

add_executable(${PROJECT_NAME} net_device.cpp)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

target_link_libraries(${PROJECT_NAME} ob::libobsensor ob::examples::common opencv::opencv)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/3rdparty/OpenCV430/lib/Windows/include/)

set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER "Examples")


# ob_install_example_target(${PROJECT_NAME})
# ob_install_example_current_source("/cpp/Sample-${PROJECT_NAME}")