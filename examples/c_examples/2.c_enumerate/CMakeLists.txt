cmake_minimum_required(VERSION 3.1.15)

project(ob_enumerate_c)
add_executable( ob_enumerate_c
    enumerate.c
)

target_link_libraries(ob_enumerate_c OrbbecSDK)

if(USE_PROJECT_FOLDERS)
    set_target_properties(ob_enumerate_c PROPERTIES FOLDER "Examples_c")
    set_target_properties(ob_enumerate_c PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
endif()

# ob_install_example_target(ob_enumerate_c)
# ob_install_example_current_source("/c/2.c_enumerate")