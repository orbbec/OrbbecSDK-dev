

cmake_minimum_required(VERSION 3.1.15)
project(ob_hdr)

add_executable(ob_hdr hdr.cpp)

set_property(TARGET ob_hdr PROPERTY CXX_STANDARD 11)
target_link_libraries(ob_hdr OrbbecSDK ${OpenCV_LIBS})
set_target_properties(ob_hdr PROPERTIES FOLDER "Examples")
# ob_install_example_target(ob_hdr) ob_install_example_current_source("/cpp/3.advanced.hdr")
