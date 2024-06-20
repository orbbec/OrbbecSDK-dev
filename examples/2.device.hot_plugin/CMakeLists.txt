#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-HotPlugin)

add_executable(HotPlugin HotPlugin.cpp)

set_property(TARGET HotPlugin PROPERTY CXX_STANDARD 11)
target_link_libraries(HotPlugin OrbbecSDK   )
IF(USE_PROJECT_FOLDERS)
set_target_properties (HotPlugin PROPERTIES FOLDER "Samples")
ENDIF()
# ob_install_example_target(HotPlugin)
# ob_install_example_current_source("/cpp/Sample-HotPlugin")
