#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(ob_quick_start)

add_executable(ob_quick_start quick_start.cpp)

set_property(TARGET ob_quick_start PROPERTY CXX_STANDARD 11)
target_link_libraries(ob_quick_start ob::openobsdk ob::examples::common opencv::opencv)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/3rdparty/OpenCV430/lib/Windows/include/)
set_target_properties (ob_quick_start PROPERTIES FOLDER "Examples")

# ob_install_example_target(ob_quick_start)
# ob_install_example_current_source("/cpp/3.advanced.post_processing")
