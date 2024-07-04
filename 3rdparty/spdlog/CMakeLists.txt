cmake_minimum_required(VERSION 3.5)

add_subdirectory(src)
ob_source_group(spdlog::spdlog)
set_target_properties(spdlog PROPERTIES FOLDER "dependencies")
