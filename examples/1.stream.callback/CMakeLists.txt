#  minimum required cmake version: 3.5 support vs2019

cmake_minimum_required(VERSION 3.5)
project(ob_callback)
include_directories(${CMAKE_SOURCE_DIR}/include/libobsensor)
add_executable(${PROJECT_NAME} callback.cpp)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} ob::openobsdk ob::examples::common ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/3rdparty/OpenCV430/lib/Windows/include/)
set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER "examples")
