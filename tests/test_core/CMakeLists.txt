cmake_minimum_required(VERSION 3.1.15)

add_executable(test_core test_core.cpp)
target_link_libraries(test_core PRIVATE ob::core ob::shared)
set_target_properties(test_core PROPERTIES FOLDER "test")
