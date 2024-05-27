cmake_minimum_required(VERSION 3.1.15)

add_executable(frame_test frame_test.cpp)
target_link_libraries(frame_test PRIVATE ob::openobsdk)
set_target_properties(frame_test PROPERTIES FOLDER "tests")
