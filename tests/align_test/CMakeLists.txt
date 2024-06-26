cmake_minimum_required(VERSION 3.1.15)

add_executable(align_test align_test.cpp ../../src/filter/public_filters/AlignImpl.hpp ../../src/filter/public_filters/AlignImpl.cpp)
target_include_directories(align_test PRIVATE ../../src/filter/public_filters/)
target_link_libraries(align_test PRIVATE ob::openobsdk ob::shared)
set_target_properties(align_test PROPERTIES FOLDER "tests")
