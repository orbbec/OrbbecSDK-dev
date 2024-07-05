#  minimum required cmake version: 3.5 support vs2019

cmake_minimum_required(VERSION 3.5)
project(ob_sync_align)

add_executable(${PROJECT_NAME} sync_align.cpp)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} ob::openobsdk ob::examples::common ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/3rdparty/OpenCV430/lib/Windows/include/)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "examples")

# ob_install_example_target(${PROJECT_NAME})
# ob_install_example_current_source("/cpp/Sample-${PROJECT_NAME}")