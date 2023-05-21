cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

FetchContent_Declare(
    googlebenchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG        v1.7.1
)
set (BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googlebenchmark)

FetchContent_declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        release-1.12.0
)

set (gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
include(GoogleTest)

include_directories(
    ${MAVLINK_INCLUDE_DIR}
    ${LWMAVSDK_INCLUDE_DIR}
)

link_libraries(
    lwmavsdk
)

add_executable(
    main-posix-uart-decoder
    main-posix-uart-decoder.cc
)

add_executable(
    main-posix-udp-decoder
    main-posix-udp-decoder.cc
)

#
# - GTests
#

add_executable(
    test_posix_uart
    test_posix_serial.cc
)

target_link_libraries(
    test_posix_uart
    PRIVATE
    util
    GTest::gtest_main
    GTest::gmock
)

gtest_discover_tests(test_posix_uart)

#
# --
#
