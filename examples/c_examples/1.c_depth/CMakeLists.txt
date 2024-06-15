cmake_minimum_required(VERSION 3.1.15)

project(ob_depth_c)
add_executable( ob_depth_c
    depth.c
)

target_link_libraries(ob_depth_c OrbbecSDK ${OpenCV_LIBS})

if(USE_PROJECT_FOLDERS)
    set_target_properties(ob_depth_c PROPERTIES FOLDER "Examples_c")
    set_target_properties(ob_depth_c PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
endif()

# ob_install_example_target(ob_depth_c)
# ob_install_example_current_source("/c/1.c_depth")