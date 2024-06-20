cmake_minimum_required(VERSION 3.1.15)

target_sources(
    ${OB_TARGET_DEVICE}
    PRIVATE ${CMAKE_CURRENT_LIST_DIR}/G330DeviceInfo.cpp
            ${CMAKE_CURRENT_LIST_DIR}/G330DeviceInfo.hpp
            ${CMAKE_CURRENT_LIST_DIR}/G330Device.cpp
            ${CMAKE_CURRENT_LIST_DIR}/G330Device.hpp
            ${CMAKE_CURRENT_LIST_DIR}/G330Device.cpp
            ${CMAKE_CURRENT_LIST_DIR}/G330Device.hpp
            ${CMAKE_CURRENT_LIST_DIR}/G330MetadataTypes.hpp
            ${CMAKE_CURRENT_LIST_DIR}/G330MetadataParser.hpp
            ${CMAKE_CURRENT_LIST_DIR}/G330MetadataParser.cpp
            ${CMAKE_CURRENT_LIST_DIR}/G330timestampCalculator.cpp
            ${CMAKE_CURRENT_LIST_DIR}/G330timestampCalculator.hpp
            ${CMAKE_CURRENT_LIST_DIR}/G330AlgParamManager.cpp
            ${CMAKE_CURRENT_LIST_DIR}/G330AlgParamManager.hpp
)
