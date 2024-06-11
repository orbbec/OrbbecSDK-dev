cmake_minimum_required(VERSION 3.1.15)
project(c_quick_start)

add_executable(ob_quick_start_c quick_start.c)
target_link_libraries(ob_quick_start_c PRIVATE ob::openobsdk)
set_target_properties(ob_quick_start_c PROPERTIES FOLDER "examples")