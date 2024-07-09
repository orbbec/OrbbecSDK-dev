
cmake_minimum_required(VERSION 3.5)
project(ob_hot_plugin)

add_executable(${PROJECT_NAME} hot_plugin.cpp)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} ob::openobsdk ob::examples::common ${OpenCV_LIBS})
set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER "examples")
# ob_install_example_target(${PROJECT_NAME})
# ob_install_example_current_source("/cpp/Sample-${PROJECT_NAME}")
