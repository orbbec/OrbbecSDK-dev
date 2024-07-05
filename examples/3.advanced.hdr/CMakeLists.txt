

cmake_minimum_required(VERSION 3.5)
project(ob_hdr)

add_executable(ob_hdr hdr.cpp)

set_property(TARGET ob_hdr PROPERTY CXX_STANDARD 11)
target_link_libraries(ob_hdr ob::openobsdk ob::examples::common ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/3rdparty/OpenCV430/lib/Windows/include/)
set_target_properties(ob_hdr PROPERTIES FOLDER "examples")
# ob_install_example_target(ob_hdr) ob_install_example_current_source("/cpp/3.advanced.hdr")
