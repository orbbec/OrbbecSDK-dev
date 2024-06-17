

cmake_minimum_required(VERSION 3.1.15)
project(ob_point_cloud)

add_executable(ob_point_cloud point_cloud.cpp)

set_property(TARGET ob_point_cloud PROPERTY CXX_STANDARD 11)
target_link_libraries(ob_point_cloud OrbbecSDK)

set_target_properties(ob_point_cloud PROPERTIES FOLDER "Examples")

# ob_install_example_target(ob_point_cloud) ob_install_example_current_source("/cpp/3.advanced.point_cloud")
