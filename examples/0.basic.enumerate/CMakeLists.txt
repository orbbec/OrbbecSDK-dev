#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(ob_enumerate)

add_executable(ob_enumerate enumerate.cpp)

set_property(TARGET ob_enumerate PROPERTY CXX_STANDARD 11)
target_link_libraries(ob_enumerate ob::openobsdk ob::examples::common ${OpenCV_LIBS})
set_target_properties (ob_enumerate PROPERTIES FOLDER "Examples")

# ob_install_example_target(ob_enumerate)
# ob_install_example_current_source("/cpp/0.basic.enumerate")
