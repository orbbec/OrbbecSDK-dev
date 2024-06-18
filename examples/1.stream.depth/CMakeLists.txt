

cmake_minimum_required(VERSION 3.1.15)
project(ob_depth)

add_executable(${PROJECT_NAME} depth.cpp)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

target_link_libraries(${PROJECT_NAME} ob::openobsdk ob::examples::common)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Examples")

if(OpenCV_FOUND)
    target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
endif()