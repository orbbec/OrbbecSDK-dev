

cmake_minimum_required(VERSION 3.1.15)
project(ob_preset)

add_executable(ob_preset preset.cpp)

set_property(TARGET ob_preset PROPERTY CXX_STANDARD 11)
target_link_libraries(ob_preset ob::openobsdk ob::examples::common ${OpenCV_LIBS})
set_target_properties(ob_preset PROPERTIES FOLDER "Examples")

# ob_install_example_target(ob_preset) ob_install_example_current_source("/cpp/3.advanced.preset")
