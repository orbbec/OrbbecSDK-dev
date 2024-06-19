#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-HotPlugin)

add_executable(HotPlugin HotPlugin.cpp)

set_property(TARGET HotPlugin PROPERTY CXX_STANDARD 11)
target_link_libraries(HotPlugin OrbbecSDK   )

