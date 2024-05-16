macro(info msg)
    message(STATUS "Info: ${msg}")
endmacro()

macro(print_variable_value variableName)
    info("${variableName}=\${${variableName}}")
endmacro()

macro(config_cxx_flags)
    set(CMAKE_CXX_STANDARD_REQUIRED True)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
    check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

    if(COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    elseif(COMPILER_SUPPORTS_CXX0X)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    else()
        # message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
    endif()
endmacro()

macro(global_set_flags)
    add_definitions(-DELPP_THREAD_SAFE)
    add_definitions(-DELPP_NO_DEFAULT_LOG_FILE)
endmacro()

macro(print_summary)
    message(STATUS "")
    message(STATUS "=================================================================================================")
    message(STATUS "Build Infomation:")
    message(STATUS "    Rove Package Version: ${PROJECT_VERSION}")
    message(STATUS "    CMake ${CMAKE_VERSION} successfully configured ${OB_LBS_TARGET} using ${CMAKE_GENERATOR} generator")
    message(STATUS "    Installation target path: ${CMAKE_INSTALL_PREFIX}")

    if(BUILD_SHARED_LIBS)
        message(STATUS "    Building Dynamic Libraries")
    else()
        message(STATUS "    Building Static Libraries")
    endif()

    message(STATUS "    Building Type: ${CMAKE_BUILD_TYPE}")
    message(STATUS "        - Debug Library Postfix: ${CMAKE_DEBUG_POSTFIX}")
    message(STATUS "        - Release Library Postfix: ${CMAKE_RELEASE_POSTFIX}")
    message(STATUS "        - MinSizeRel Library Postfix: ${CMAKE_MINSIZEREL_POSTFIX}")
    message(STATUS "        - RelWithDebInfo Library Postfix: ${CMAKE_RELWITHDEBINFO_POSTFIX}")
    message(STATUS "C Complier Flags:")
    message(STATUS "    C_FLAGS:  =${CMAKE_C_FLAGS}")
    message(STATUS "    C_FLAGS_DEBUG:=${CMAKE_C_FLAGS_DEBUG}")
    message(STATUS "    C_FLAGS_RELEASE:=${CMAKE_C_FLAGS_RELEASE}")
    message(STATUS "    C_FLAGS_MINSIZEREL:=${CMAKE_C_FLAGS_MINSIZEREL}")
    message(STATUS "    C_FLAGS_RELWITHDEBINFO:=${CMAKE_C_FLAGS_RELWITHDEBINFO}")
    message(STATUS "C++ Complier Flags:")
    message(STATUS "    CXX_FLAGS:=${CMAKE_CXX_FLAGS}")
    message(STATUS "    CXX_FLAGS_DEBUG:=${CMAKE_CXX_FLAGS_DEBUG}")
    message(STATUS "    CXX_FLAGS_RELEASE:=${CMAKE_CXX_FLAGS_RELEASE}")
    message(STATUS "    CMAKE_EXE_LINKER_FLAGS_RELEASE:=${CMAKE_EXE_LINKER_FLAGS_RELEASE}")
    message(STATUS "    CXX_FLAGS_MINSIZEREL:=${CMAKE_CXX_FLAGS_MINSIZEREL}")
    message(STATUS "    CXX_FLAGS_RELWITHDEBINFO:=${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
endmacro()

function(global_include_append include_path)
    include_directories(${include_path})
    set_property(
        GLOBAL
        PROPERTY CORE_INC ${include_path}
        APPEND)
endfunction()

function(subdirectory_include property_name)
    get_property(core_inc_dirs GLOBAL PROPERTY ${property_name})

    foreach(child ${core_inc_dirs})
        include_directories(${child})
        message(STATUS "Include: " ${child})
    endforeach()
endfunction()

macro(generate_useful_info BRANCH_NAME COMMIT_HASH CURRENT_TIME PLATFORM PROGRAMMING_LANGUAGE)
    set(GIT_NAME git)
    find_program(GIT_COMMAND ${GIT_NAME})

    if(EXISTS ${GIT_COMMAND})
        execute_process(
            COMMAND ${GIT_COMMAND} --version
            WORKING_DIRECTORY ${ORBBEC_SDK_ROOT_DIR}
            OUTPUT_VARIABLE GIT_VERSION
            OUTPUT_STRIP_TRAILING_WHITESPACE)
        message("Found ${GIT_NAME}: ${GIT_COMMAND} (${GIT_VERSION})")
        execute_process(
            COMMAND ${GIT_COMMAND} rev-parse --short HEAD
            WORKING_DIRECTORY ${ORBBEC_SDK_ROOT_DIR}
            OUTPUT_VARIABLE GIT_COMMIT_HASH
            OUTPUT_STRIP_TRAILING_WHITESPACE)
        execute_process(
            COMMAND ${GIT_COMMAND} rev-parse --abbrev-ref HEAD
            WORKING_DIRECTORY ${ORBBEC_SDK_ROOT_DIR}
            OUTPUT_VARIABLE GIT_BRANCH
            OUTPUT_STRIP_TRAILING_WHITESPACE)
    else()
        message(FATAL_ERROR "Command ${GIT_NAME} cannot be found, so could not obtain GIT_BRANCH and GIT_COMMIT_HASH informations.")
    endif()

    string(TIMESTAMP TIME "%Y%m%d")
    set(${BRANCH_NAME} "alpha")

    if(${GIT_BRANCH} MATCHES "beta" OR ${GIT_BRANCH} MATCHES "Beta")
        set(${BRANCH_NAME} "beta")
    elseif(${GIT_BRANCH} MATCHES "master" OR ${GIT_BRANCH} MATCHES "Master")
        set(${BRANCH_NAME} "release")
    endif()

    set(${COMMIT_HASH} ${GIT_COMMIT_HASH})
    set(${CURRENT_TIME} ${TIME})

    set(${PLATFORM} ${OB_CURRENT_OS})
    set(${PROGRAMMING_LANGUAGE} "C_C++")

    if(${OB_CURRENT_OS} STREQUAL "linux")
        set(${PLATFORM} "${OB_CURRENT_OS}_x64")
    endif()
endmacro()

macro(subDirList result curdir)
    file(
        GLOB children
        RELATIVE ${curdir}
        ${curdir}/*)
    set(subdirlist "")

    foreach(child ${children})
        if(IS_DIRECTORY ${curdir}/${child})
            list(APPEND subdirlist ${child})
        endif()
    endforeach()

    set(${result} ${subdirlist})
endmacro()

# ##############################################################################################################################################################
# Add a library target to install name the Name of the library place the Place of the library will be installed!
# ##############################################################################################################################################################
macro(ob_add_library name place)
    if(OB_MASTER_PROJECT)
        if(WIN32 AND MSVC)
            get_target_property(Module_TYPE ${name} Module)

            if(Module_TYPE STREQUAL "INNERTEST" AND (NOT INSTALL_INCLUDE_INNERTEST))

            else()
                get_target_property(TARGET_TYPE ${name} TYPE)

                if(TARGET_TYPE STREQUAL "EXECUTABLE")
                    set_target_properties(${name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
                    install(TARGETS ${name} DESTINATION ${place})
                else()
                    if(Module_TYPE STREQUAL "SDKLIB")
                        install(TARGETS ${name} DESTINATION ${place})
                    else()
                        install(TARGETS ${name} RUNTIME DESTINATION ${place})
                    endif()
                endif()
            endif()
        elseif(ANDROID_NDK_TOOLCHAIN_INCLUDED OR BUILD_ANDROID)
            get_target_property(TARGET_TYPE ${name} TYPE)

            if(TARGET_TYPE STREQUAL "EXECUTABLE" AND BUILD_ANDROIDSDK_ONLY)

            else()
                get_target_property(Module_TYPE ${name} Module)

                if(Module_TYPE STREQUAL "INNERTEST" AND (NOT INSTALL_INCLUDE_INNERTEST))

                else()
                    install(TARGETS ${name} DESTINATION ${ANDROID_ABI})
                endif()
            endif()
        else()
            get_target_property(Module_TYPE ${name} Module)

            if(Module_TYPE STREQUAL "INNERTEST" AND (NOT INSTALL_INCLUDE_INNERTEST))

            else()
                install(TARGETS ${name} DESTINATION ${place})
            endif()
        endif()
    endif()
endmacro()

# ##############################################################################################################################################################
# Module can only be one of the following values SDKLIB TOOL INNERTEST OUTERTEST SAMPLE SDKLIB : Dependent libraries or Target generated OBSENSORSDK TOOL: TOOL
# Target in OBSENSORSDK,Maybe the TOOL will subdivision for innerTool and outerTool in the future. INNERTEST: inner test target in OBSENSORSDK OUTERTEST: onter
# test target in OBSENSORSDK SAMPLE: sample target in OBSENSORSDK,Maybe the SAMPLE will subdivision for innerSample and outerSample in the future.
# ##############################################################################################################################################################
macro(ob_add_target_property libname property property_value)
    set_target_properties(${libname} PROPERTIES ${property} ${property_value})
endmacro()
