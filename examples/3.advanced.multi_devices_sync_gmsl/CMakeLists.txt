

cmake_minimum_required(VERSION 3.5)
project(ob_multi_devices_sync_gmsl)

add_executable(${PROJECT_NAME} ob_multi_devices_sync_gmsl.cpp utils/cJSON.c)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} ob::OrbbecSDK ob::examples::utils)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "examples")

set(CONFIG_FILE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/syncconfig)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
install(DIRECTORY ${CONFIG_FILE_DIRECTORY} DESTINATION bin)