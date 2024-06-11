cmake_minimum_required(VERSION 2.8)

project(imu_calib_params C CXX)

set(CMAKE_CXX_STANDARD 11)

if("${CMAKE_BUILD_TYPE}" STREQUAL "")
    SET(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)

# set(gcc_like_compilers GNU Clang Intel)
add_compile_options(-fPIC)

# # warning level
# if(MSVC)
# # Force to always compile with W4
# if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]" OR CMAKE_C_FLAGS MATCHES "/W[0-4]")
# string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
# string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
# else()
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /W4")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
# endif()

# ADD_DEFINITIONS(
# -D_CRT_SECURE_NO_WARNINGS
# -D_CRT_SECURE_NO_DEPRECATE
# )
# elseif((";${gcc_like_compilers};" MATCHES ";${CMAKE_CXX_COMPILER_ID};"))
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall") # -Werror")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall") # -Werror")
# endif()
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Include)
message("========================${CMAKE_CURRENT_SOURCE_DIR}")

# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/jsoncpp-1.8.4/build/install/include)
# link_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/jsoncpp-1.8.4/build/install/lib)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/yaml-cpp-0.6.0)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/yaml-cpp-0.6.0/include)

# include_directories(${CALIBPARAM_3RDPARTY_INCLUDE_DIR}/)
# link_directories(${CALIBPARAM_3RDPARTY_LIB_DIR}/)
enable_testing()

# add_subdirectory(3rdParty)
add_subdirectory(Include)

add_subdirectory(Source)