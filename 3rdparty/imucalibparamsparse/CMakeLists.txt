cmake_minimum_required(VERSION 3.5)

add_subdirectory(src)

# remove warning as error
get_target_property(compile_options imu_params_parser COMPILE_OPTIONS)
list(REMOVE_ITEM compile_options "-Werror" "/WX" "-Wpedantic")
set_target_properties(imu_params_parser PROPERTIES COMPILE_OPTIONS "${compile_options}")

add_library(imu::imu_params_parser ALIAS imu_params_parser)
ob_source_group(imu::imu_params_parser)
set_target_properties(imu_params_parser PROPERTIES FOLDER "dependencies")
set_target_properties(yaml-cpp PROPERTIES FOLDER "dependencies")
