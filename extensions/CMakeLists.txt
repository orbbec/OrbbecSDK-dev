cmake_minimum_required(VERSION 3.5)

add_custom_target(extensions)

add_custom_command(
    TARGET extensions
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_LIST_DIR}/filters/${OB_CURRENT_OS}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/extensions/filters")


add_custom_command(
    TARGET extensions
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_LIST_DIR}/frameprocessor/${OB_CURRENT_OS}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/extensions/frameprocessor")

set_target_properties(extensions PROPERTIES FOLDER "modules")