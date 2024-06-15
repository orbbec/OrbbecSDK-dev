#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(ob_depth)

add_executable(ob_depth depth.cpp)

set_property(TARGET ob_depth PROPERTY CXX_STANDARD 11)
target_link_libraries(ob_depth OrbbecSDK ${OpenCV_LIBS})
IF(USE_PROJECT_FOLDERS)
    set_target_properties (ob_depth PROPERTIES FOLDER "Examples")
ENDIF()

# ob_install_example_target(ob_depth)
# ob_install_example_current_source("/cpp/1.stream.depth")
