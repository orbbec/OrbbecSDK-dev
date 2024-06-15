#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(ob_hdr)

add_executable(ob_hdr hdr.cpp)

set_property(TARGET ob_hdr PROPERTY CXX_STANDARD 11)
target_link_libraries(ob_hdr OrbbecSDK ${OpenCV_LIBS})

IF(USE_PROJECT_FOLDERS)
    set_target_properties (ob_hdr PROPERTIES FOLDER "Examples")
ENDIF()

# ob_install_example_target(ob_hdr)
# ob_install_example_current_source("/cpp/3.advanced.hdr")