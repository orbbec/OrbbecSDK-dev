cmake_minimum_required(VERSION 3.1.15)

if(INSTALL_MODULE_SHARED)
    set(SPDLOG_INSTALL ON)
endif()

add_subdirectory(src)
ob_source_group(spdlog::spdlog)
set_target_properties(spdlog PROPERTIES FOLDER "dependencies")
