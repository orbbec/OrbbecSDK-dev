
cmake_minimum_required(VERSION 3.5)
project(ob_net_device)

add_executable(${PROJECT_NAME} net_device.cpp)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

target_link_libraries(${PROJECT_NAME} ob::openobsdk ob::examples::utils)

set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER "examples")

install(TARGETS ob_net_device RUNTIME DESTINATION bin)
