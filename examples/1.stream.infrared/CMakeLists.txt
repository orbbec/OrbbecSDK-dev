#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(ob_infrared)

add_executable(ob_infrared infrared.cpp)

set_property(TARGET ob_infrared PROPERTY CXX_STANDARD 11)
target_link_libraries(ob_infrared ob::libobsensor ob::examples::common opencv::opencv)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/3rdparty/OpenCV430/lib/Windows/include/)
set_target_properties (ob_infrared PROPERTIES FOLDER "Examples")


# ob_install_example_target(ob_infrared)
# ob_install_example_current_source("/cpp/1.stream.infrared")