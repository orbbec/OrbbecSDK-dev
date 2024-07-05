cmake_minimum_required(VERSION 3.5)

project(ob_enumerate_c)
add_executable(ob_enumerate_c enumerate.c)

target_link_libraries(ob_enumerate_c PRIVATE ob::openobsdk ob::examples::common)

set_target_properties(ob_enumerate_c PROPERTIES FOLDER "examples_c")
set_target_properties(ob_enumerate_c PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

# ob_install_example_target(ob_enumerate_c) ob_install_example_current_source("/c/2.c_enumerate")
