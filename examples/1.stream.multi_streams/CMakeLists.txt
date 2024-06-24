

cmake_minimum_required(VERSION 3.1.15)
project(ob_multi_streams)

add_executable(ob_multi_streams multi_streams.cpp)

set_property(TARGET ob_multi_streams PROPERTY CXX_STANDARD 11)
target_link_libraries(ob_multi_streams ob::libobsensor ob::examples::common opencv::opencv)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/3rdparty/OpenCV430/lib/Windows/include/)
set_target_properties(ob_multi_streams PROPERTIES FOLDER "Examples")

# ob_install_example_target(ob_multi_streams) ob_install_example_current_source("/cpp/1.stream.multi_streams")
