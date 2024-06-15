#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-PointCloud)

add_executable(PointCloud PointCloud.cpp)

set_property(TARGET PointCloud PROPERTY CXX_STANDARD 11)
target_link_libraries(PointCloud OrbbecSDK   )

