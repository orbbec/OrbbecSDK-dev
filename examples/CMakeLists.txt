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
add_subdirectory(common)

add_subdirectory(c_examples)

add_subdirectory(0.basic.enumerate)
add_subdirectory(0.basic.quick_start)
add_subdirectory(1.stream.color)
add_subdirectory(1.stream.depth)
add_subdirectory(1.stream.imu)
add_subdirectory(1.stream.infrared)
add_subdirectory(1.stream.multi_streams)
# add_subdirectory(3.advanced.hdr)
# add_subdirectory(3.advanced.point_cloud)
add_subdirectory(3.advanced.post_processing)
add_subdirectory(3.advanced.preset)


