cmake_minimum_required(VERSION 3.1.15)
project(c_hello)

set(CMAKE_CXX_STANDARD 11)

add_executable(ob_hello_c hello.c)

target_link_libraries(ob_hello_c PRIVATE ob::openobsdk)