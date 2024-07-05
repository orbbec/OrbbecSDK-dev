

cmake_minimum_required(VERSION 3.5)
project(ob_imu)

add_executable(ob_imu imu.cpp)

set_property(TARGET ob_imu PROPERTY CXX_STANDARD 11)
target_link_libraries(ob_imu ob::openobsdk ob::examples::common)

set_target_properties(ob_imu PROPERTIES FOLDER "examples")

# ob_install_example_target(ob_imu) ob_install_example_current_source("/cpp/1.stream.imu")
