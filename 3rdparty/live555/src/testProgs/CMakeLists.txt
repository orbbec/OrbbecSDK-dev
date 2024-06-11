cmake_minimum_required(VERSION 3.0.0)

add_executable(testRtspClient testRTSPClient.cpp)
target_include_directories(testRtspClient
    PRIVATE
    ${LIVE555_INCLUDE_DIRS}
)
target_link_libraries(testRtspClient PRIVATE live555)


# if (WIN32)
#     target_link_libraries(testRtspClient PRIVATE Ws2_32)
# endif ()