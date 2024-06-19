

cmake_minimum_required(VERSION 3.1.15)
project(ob_depth)

add_executable(${PROJECT_NAME} depth.cpp)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

target_link_libraries(${PROJECT_NAME} ob::openobsdk ob::examples::common opencv::opencv)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/3rdparty/OpenCV430/lib/Windows/include/)

# set(OPENCV_LIB_PATH "${CMAKE_SOURCE_DIR}/3rdparty/OpenCV430/lib/Windows/lib/win_x64/")
# link_directories(OPENCV_LIB_PATH)
# file(GLOB LIBRARY_FILES "${OPENCV_LIB_PATH}/*.dll")

# target_link_libraries(${PROJECT_NAME} ${LIBRARY_FILES})


set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Examples")

