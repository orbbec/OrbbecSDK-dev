#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-ImuReader)

add_executable(ImuReader ImuReader.cpp)

set_property(TARGET ImuReader PROPERTY CXX_STANDARD 11)
target_link_libraries(ImuReader OrbbecSDK)