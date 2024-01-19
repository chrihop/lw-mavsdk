cmake_minimum_required(VERSION 3.24 FATAL_ERROR)

project(lw-mavsdk)

option (BUILD_FOR "The target system to link with" "posix")
set_property(CACHE BUILD_FOR PROPERTY STRINGS
    "posix" "baremetal" "certikos_user" "linux_kernel")

include(CheckCCompilerFlag)
include(ProcessorCount)
ProcessorCount(N_JOBS)
include(FindPython3)
message(STATUS "Python: " ${Python3_EXECUTABLE})
include(FetchContent)

set (CMAKE_C_STANDARD 11)
set (CMAKE_CXX_STANDARD 17)

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif()

set (LWMAVSDK_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set (LWMAVSDK_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR})
set (LWMAVSDK_INCLUDE_DIR ${LWMAVSDK_SOURCE_DIR}/inc)

set (LWMAVSDK_FLIGHT_CONTROL_DIR ${LWMAVSDK_SOURCE_DIR}/flight_control)
set (LWMAVSDK_FLIGHT_CONTROL_INC ${LWMAVSDK_SOURCE_DIR}/flight_control)
set (LWMAVSDK_FLIGHT_CONTROL_BUILD ${LWMAVSDK_BUILD_DIR}/flight_control)

set (LWMAVSDK_C_DEFINITIONS "")

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    list(APPEND LWMAVSDK_C_DEFINITIONS "DEBUG_BUILD")
endif()

if (BUILD_FOR STREQUAL "posix")
    message (STATUS "lw-mavsdk: posix build")
    set (LWMAVSDK_HAS_TEST true)
    list(APPEND LWMAVSDK_C_DEFINITIONS "POSIX_LIBC")
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_options(-O0 -g -ggdb)
        add_compile_options(-fsanitize=address,undefined -fno-sanitize=alignment)
        add_link_options(-fsanitize=address,undefined -fno-sanitize=alignment)
        add_link_options(-rdynamic)
    endif()
elseif (BUILD_FOR STREQUAL "baremetal")
    message (STATUS "lw-mavsdk: baremetal build")
    list(APPEND LWMAVSDK_C_DEFINITIONS "BAREMETAL")

    add_compile_options(-nostdlib -nostdinc -ffreestanding -fno-builtin
        -fno-common -fno-exceptions -fno-stack-protector
        -fno-unwind-tables -fno-asynchronous-unwind-tables -fno-strict-aliasing
        -fno-omit-frame-pointer -fno-strict-overflow -fno-delete-null-pointer-checks
        -fno-PIE -fno-pic -fno-pie -fno-pic -fno-stack-protector -fno-unwind-tables
        -fno-asynchronous-unwind-tables -fno-exceptions -fno-omit-frame-pointer
        -fno-delete-null-pointer-checks)

    add_link_options(
        -nostdlib -nostdinc -ffreestanding -fno-builtin
        -fno-common -fno-exceptions -fno-stack-protector
        -fno-unwind-tables -fno-asynchronous-unwind-tables -fno-strict-aliasing
        -fno-omit-frame-pointer -fno-strict-overflow -fno-delete-null-pointer-checks
        -fno-PIE -fno-pic -fno-pie -fno-pic -fno-stack-protector -fno-unwind-tables
        -fno-asynchronous-unwind-tables -fno-exceptions -fno-omit-frame-pointer
        -fno-delete-null-pointer-checks
    )

    execute_process(COMMAND ${CMAKE_C_COMPILER} -print-file-name=libgcc.a
        OUTPUT_VARIABLE C_LIBGCC_FILE
        OUTPUT_STRIP_TRAILING_WHITESPACE)

    execute_process(COMMAND ${CMAKE_C_COMPILER} -print-file-name=libm.a
        OUTPUT_VARIABLE C_LIBM_FILE
        OUTPUT_STRIP_TRAILING_WHITESPACE)

    link_libraries(
        ${C_LIBGCC_FILE}
        ${C_LIBM_FILE}
    )
elseif (BUILD_FOR STREQUAL "certikos_user")
    message(STATUS "lw-mavsdk: certikos user build")
    list(APPEND LWMAVSDK_C_DEFINITIONS "CERTIKOS_USER")
else()
    message(FATAL_ERROR "lw-mavsdk: unknown target")
endif()

add_compile_definitions(${LWMAVSDK_C_DEFINITIONS})
string(REPLACE ";" " " LWMAVSDK_C_DEFINITIONS_STR "${LWMAVSDK_C_DEFINITIONS}")
message(STATUS "lw-mavsdk: c definitions ${LWMAVSDK_C_DEFINITIONS_STR}")

#
# MAVLINK
#
FetchContent_Declare(
    mavlink
    GIT_REPOSITORY https://github.com/mavlink/mavlink.git
    GIT_TAG master
    OVERRIDE_FIND_PACKAGE
)

set (MAVLINK_DIALECT "ardupilotmega" CACHE STRING "" FORCE)
set (MAVLINK_VERSION "2.0" CACHE STRING "" FORCE)

FetchContent_MakeAvailable(mavlink)

message(STATUS "lw-mavsdk: mavlink source dir ${mavlink_SOURCE_DIR}")
set (MAVLINK_SOURCE_DIR "${mavlink_SOURCE_DIR}" CACHE STRING "" FORCE)
set (MAVLINK_INCLUDE_DIR "${mavlink_BINARY_DIR}/include" CACHE STRING "" FORCE)

add_custom_command(
    OUTPUT ${MAVLINK_DIALECT}-${MAVLINK_VERSION}-stamp
    COMMAND PYTHONPATH=$ENV{PYTHONPATH}:${MAVLINK_SOURCE_DIR}
            ${Python3_EXECUTABLE} -m pymavlink.tools.mavgen
            --lang=C --wire-protocol=${MAVLINK_VERSION}
            --output=${MAVLINK_INCLUDE_DIR}/v${MAVLINK_VERSION}
            ${MAVLINK_SOURCE_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT}.xml
            COMMAND touch ${MAVLINK_DIALECT}-${MAVLINK_VERSION}-stamp
)

add_custom_target(${MAVLINK_DIALECT}-${MAVLINK_VERSION}
            DEPENDS ${MAVLINK_DIALECT}-${MAVLINK_VERSION}-stamp)

add_custom_target(mavlink-headers
    DEPENDS ${MAVLINK_DIALECT}-${MAVLINK_VERSION})

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
else()
    set (LWMAVSDK_INC_DIRS
        ${LWMAVSDK_INCLUDE_DIR}
        ${LWMAVSDK_FLIGHT_CONTROL_INC}
        ${MAVLINK_INCLUDE_DIR}
        PARENT_SCOPE)

    set (LWMAVSDK_C_DEFINITIONS
        ${LWMAVSDK_C_DEFINITIONS}
        PARENT_SCOPE)
endif()

add_compile_options(-Wno-address-of-packed-member)
add_compile_options(-Wno-missing-field-initializers)

#
# ---
#

add_subdirectory(src)

if (BUILD_FOR STREQUAL "posix")
    add_subdirectory(tests)
endif()

# add_subdirectory(tools)
