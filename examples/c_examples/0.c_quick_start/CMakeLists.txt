cmake_minimum_required(VERSION 3.1.15)
project(c_quick_start)

set(CMAKE_CXX_STANDARD 11)

add_executable(ob_quick_start_c quick_start.c)

target_link_libraries(ob_quick_start_c PRIVATE ob::openobsdk)