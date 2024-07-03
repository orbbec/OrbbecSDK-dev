cmake_minimum_required(VERSION 3.5)

add_subdirectory(thirdParty/yaml-cpp-0.6.0)

set(IMU_PARAMS_PARSER_SOURCE_FILES
        ${CMAKE_CURRENT_LIST_DIR}/Source/imu_calibration_params_parser.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Source/imu_calibration_params_yaml.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Source/libcrc-2.0/crc8.c
        ${CMAKE_CURRENT_LIST_DIR}/Source/libcrc-2.0/crc16.c
        ${CMAKE_CURRENT_LIST_DIR}/Source/libcrc-2.0/crc32.c
        ${CMAKE_CURRENT_LIST_DIR}/Source/libcrc-2.0/crcccitt.c
        ${CMAKE_CURRENT_LIST_DIR}/Source/libcrc-2.0/crcdnp.c
        ${CMAKE_CURRENT_LIST_DIR}/Source/libcrc-2.0/crckrmit.c
        ${CMAKE_CURRENT_LIST_DIR}/Source/libcrc-2.0/crcsick.c
        ${CMAKE_CURRENT_LIST_DIR}/Source/libcrc-2.0/nmea-chk.c
)
set(IMU_PARAMS_PARSER_INCLUDE_FILES ${CMAKE_CURRENT_LIST_DIR}/Include/imu_calibration_params.h
                  ${CMAKE_CURRENT_LIST_DIR}/Include/imu_calibration_params_parser.h
                  ${CMAKE_CURRENT_LIST_DIR}/Include/version.h)


add_library(imu_params_parser STATIC ${IMU_PARAMS_PARSER_SOURCE_FILES} ${IMU_PARAMS_PARSER_INCLUDE_FILES})
target_include_directories(imu_params_parser PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Include)
target_include_directories(imu_params_parser PUBLIC ${CMAKE_CURRENT_LIST_DIR}/thirdParty/yaml-cpp-0.6.0/include)
target_include_directories(imu_params_parser PUBLIC ${CMAKE_CURRENT_LIST_DIR}/thirdParty/eigen3)

# include_directories(${CMAKE_CURRENT_LIST_DIR}/Include)
# include_directories(${CMAKE_CURRENT_LIST_DIR}/thirdParty/yaml-cpp-0.6.0/include)

target_link_libraries(imu_params_parser yaml-cpp)

get_target_property(compile_options yaml-cpp COMPILE_OPTIONS)
list(REMOVE_ITEM compile_options "-Werror" "/WX" "-Wpedantic")
set_target_properties(yaml-cpp PROPERTIES COMPILE_OPTIONS "${compile_options}")
