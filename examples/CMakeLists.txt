cmake_minimum_required(VERSION 3.1.15)

file(GLOB subdirectories RELATIVE ${CMAKE_CURRENT_LIST_DIR} "*")



# foreach(subdir ${subdirectories})
#     if(IS_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/${subdir})
#         add_subdirectory(${subdir})
#     endif()
# endforeach()

# set(OpenCV_DIR "${CMAKE_SOURCE_DIR}/3rdparty/OpenCV430/Windows")
# find_package(OpenCV REQUIRED)

add_subdirectory(${OB_3RDPARTY_DIR}/OpenCV430 opencv)
# message("***opencv: ${OB_3RDPARTY_DIR}/OpenCV430")

add_subdirectory(1.stream.depth)
add_subdirectory(common)
add_subdirectory(c_examples)


# if(UNIX)
#     FILE(GLOB OpenCV_LIBS_FILE ${OpenCV_LIBRARY_DIRS}/*)
#     install(FILES ${OpenCV_LIBS_FILE} DESTINATION ${OB_EXAMPLE_BIN_INSTALL_DIR})
# else()
#     install(FILES "${OpenCV_LIBRARY_DIRS}/${OpenCV_LIBS}" DESTINATION ${OB_EXAMPLE_BIN_INSTALL_DIR})
# endif()

