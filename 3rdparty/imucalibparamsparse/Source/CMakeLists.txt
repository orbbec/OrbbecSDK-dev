set(SOURCE_FILES
        imu_calibration_params_parser.cpp
        imu_calibration_params_yaml.cpp
        libcrc-2.0/crc8.c
        libcrc-2.0/crc16.c
        libcrc-2.0/crc32.c
        libcrc-2.0/crcccitt.c
        libcrc-2.0/crcdnp.c
        libcrc-2.0/crckrmit.c
        libcrc-2.0/crcsick.c
        libcrc-2.0/nmea-chk.c
)
set(INCLUDE_FILES ${CMAKE_SOURCE_DIR}/Include/imu_calibration_params.h
                  ${CMAKE_SOURCE_DIR}/Include/imu_calibration_params_parser.h
                  ${CMAKE_SOURCE_DIR}/Include/version.h)
add_library(imu_calib_params_parser_static STATIC ${SOURCE_FILES} ${INCLUDE_FILES})
add_library(imu_calib_params_parser_shared SHARED ${SOURCE_FILES} ${INCLUDE_FILES})

if(WIN32)
        if(CMAKE_CL_64)
                set_target_properties(imu_calib_params_parser_static PROPERTIES
                        OUTPUT_NAME "imu_calib_params_parser"
                        CLEAN_DIRECT_OUTPUT 1
                        ARCHIVE_OUTPUT_DIRECTORY staticlib/x64)

                install(TARGETS imu_calib_params_parser_static
                        ARCHIVE DESTINATION staticlib/x64
                )

                set_target_properties(imu_calib_params_parser_shared PROPERTIES
                        OUTPUT_NAME "imu_calib_params_parser"
                        CLEAN_DIRECT_OUTPUT 1
                        COMPILE_FLAGS "-DVCAL_EXPORTS"
                        RUNTIME_OUTPUT_DIRECTORY bin/x64
                        ARCHIVE_OUTPUT_DIRECTORY lib/x64)

                install(TARGETS imu_calib_params_parser_shared
                        RUNTIME DESTINATION bin/x64
                        ARCHIVE DESTINATION lib/x64
                )
        else(CMAKE_CL_64)
                set_target_properties(imu_calib_params_parser_static PROPERTIES
                        OUTPUT_NAME "imu_calib_params_parser"
                        CLEAN_DIRECT_OUTPUT 1
                        ARCHIVE_OUTPUT_DIRECTORY staticlib/x86
                )

                install(TARGETS imu_calib_params_parser_static
                        ARCHIVE DESTINATION staticlib/x86
                )

                set_target_properties(imu_calib_params_parser_shared PROPERTIES
                        OUTPUT_NAME "imu_calib_params_parser"
                        CLEAN_DIRECT_OUTPUT 1
                        COMPILE_FLAGS "-DVCAL_EXPORTS"
                        RUNTIME_OUTPUT_DIRECTORY bin/x86
                        ARCHIVE_OUTPUT_DIRECTORY lib/x86)

                install(TARGETS imu_calib_params_parser_shared
                        RUNTIME DESTINATION bin/x86
                        ARCHIVE DESTINATION lib/x86
                )
        endif(CMAKE_CL_64)

else(WIN32)
        set_target_properties(imu_calib_params_parser_static PROPERTIES
                OUTPUT_NAME "imu_calib_params_parser"
                CLEAN_DIRECT_OUTPUT 1)
        install(TARGETS imu_calib_params_parser_static
                LIBRARY DESTINATION lib
                ARCHIVE DESTINATION lib
        )

        set_target_properties(imu_calib_params_parser_shared PROPERTIES
                OUTPUT_NAME "imu_calib_params_parser"
                CLEAN_DIRECT_OUTPUT 1)
        install(TARGETS imu_calib_params_parser_shared
                LIBRARY DESTINATION lib
                ARCHIVE DESTINATION lib
        )
endif(WIN32)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Include)

target_link_libraries(imu_calib_params_parser_static
        yaml-cpp
)

target_link_libraries(imu_calib_params_parser_shared
        yaml-cpp
)
