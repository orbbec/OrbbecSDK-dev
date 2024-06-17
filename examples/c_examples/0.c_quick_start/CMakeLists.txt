cmake_minimum_required(VERSION 3.1.15)

set(CMAKE_CXX_STANDARD 11)
project(ob_quick_start_c)
add_executable(ob_quick_start_c quick_start.c)

target_link_libraries(ob_quick_start_c PRIVATE ob::openobsdk ob::examples::common)

set_target_properties(ob_quick_start_c PROPERTIES FOLDER "Examples_c")
set_target_properties(ob_quick_start_c PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
# ob_install_example_target(ob_quick_start_c) ob_install_example_current_source("/c/0.c_quick_start")
