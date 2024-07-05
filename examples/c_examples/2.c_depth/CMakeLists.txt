cmake_minimum_required(VERSION 3.5)

project(ob_depth_c)
add_executable(ob_depth_c depth.c)

target_link_libraries(ob_depth_c PRIVATE ob::openobsdk ob::examples::common)

set_target_properties(ob_depth_c PROPERTIES FOLDER "examples_c")
set_target_properties(ob_depth_c PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

# ob_install_example_target(ob_depth_c) ob_install_example_current_source("/c/1.c_depth")
