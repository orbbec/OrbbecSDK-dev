#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-NetDevice)

add_executable(NetDevice NetDevice.cpp)
set_property(TARGET NetDevice PROPERTY CXX_STANDARD 11)

target_link_libraries(NetDevice ob::openobsdk ob::examples::common opencv::opencv)
target_include_directories(NetDevice PUBLIC ${CMAKE_SOURCE_DIR}/3rdparty/OpenCV430/lib/Windows/include/)

IF(USE_PROJECT_FOLDERS)
    set_target_properties (NetDevice PROPERTIES FOLDER "Samples")
ENDIF()

# ob_install_example_target(NetDevice)
# ob_install_example_current_source("/cpp/Sample-NetDevice")