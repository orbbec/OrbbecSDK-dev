cmake_minimum_required(VERSION 3.1.15)

add_executable(align_test align_test.cpp)
target_link_libraries(align_test PRIVATE ob::libobsensor)
set_target_properties(align_test PROPERTIES FOLDER "tests")
