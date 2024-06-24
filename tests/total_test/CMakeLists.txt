cmake_minimum_required(VERSION 3.1.15)

add_executable(total_test total_test.cpp)
target_link_libraries(total_test PRIVATE ob::libobsensor)
set_target_properties(total_test PROPERTIES FOLDER "tests")
