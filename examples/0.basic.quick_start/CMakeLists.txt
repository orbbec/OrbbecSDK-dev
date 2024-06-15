#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(ob_quick_start)

add_executable(ob_quick_start quick_start.cpp)

set_property(TARGET ob_quick_start PROPERTY CXX_STANDARD 11)
target_link_libraries(ob_quick_start openobsdk ${OpenCV_LIBS})
IF(USE_PROJECT_FOLDERS)
    set_target_properties (ob_quick_start PROPERTIES FOLDER "Examples")
ENDIF()

# ob_install_example_target(ob_quick_start)
# ob_install_example_current_source("/cpp/3.advanced.post_processing")
