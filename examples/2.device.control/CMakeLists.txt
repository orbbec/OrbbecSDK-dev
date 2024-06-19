#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-SensorControl)

add_executable(SensorControl SensorControl.cpp)

set_property(TARGET SensorControl PROPERTY CXX_STANDARD 11)
target_link_libraries(SensorControl OrbbecSDK )
IF(USE_PROJECT_FOLDERS)
    set_target_properties (SensorControl PROPERTIES FOLDER "Samples")
ENDIF()

ob_install_example_target(SensorControl)
ob_install_example_current_source("/cpp/Sample-SensorControl")