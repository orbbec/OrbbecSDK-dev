

cmake_minimum_required(VERSION 3.1.15)
project(ob_multi_streams)

add_executable(ob_multi_streams multi_streams.cpp)

set_property(TARGET ob_multi_streams PROPERTY CXX_STANDARD 11)
target_link_libraries(ob_multi_streams ob::openobsdk ob::examples::common ${OpenCV_LIBS})
set_target_properties(ob_multi_streams PROPERTIES FOLDER "Examples")

# ob_install_example_target(ob_multi_streams) ob_install_example_current_source("/cpp/1.stream.multi_streams")
