
cmake_minimum_required(VERSION 3.1.15)
project(ob_multi_device)

add_executable(${PROJECT_NAME} multi_device.cpp)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} ob::openobsdk ob::examples::common ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/3rdparty/OpenCV430/lib/Windows/include/)
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "examples")

