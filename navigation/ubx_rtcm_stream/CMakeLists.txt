cmake_minimum_required(VERSION 3.16)
project(robodog_ubx_rtcm_stream LANGUAGES CXX)

option(ROBODOG_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(ROBODOG_BUILD_FUZZER "Build the libFuzzer byte-stream target (Clang only)" OFF)

add_library(robodog_ubx_rtcm_stream src/stream_decoder.cpp)
target_include_directories(robodog_ubx_rtcm_stream PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(robodog_ubx_rtcm_stream PUBLIC cxx_std_17)
target_compile_options(robodog_ubx_rtcm_stream PRIVATE -Wall -Wextra -Wpedantic -Wconversion)

if(ROBODOG_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(robodog_ubx_rtcm_stream PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(robodog_ubx_rtcm_stream PUBLIC -fsanitize=address,undefined)
endif()

enable_testing()
add_executable(stream_decoder_tests tests/test_stream_decoder.cpp)
target_link_libraries(stream_decoder_tests PRIVATE robodog_ubx_rtcm_stream)
target_compile_options(stream_decoder_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME stream_decoder_tests COMMAND stream_decoder_tests)

if(ROBODOG_BUILD_FUZZER)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(FATAL_ERROR "ROBODOG_BUILD_FUZZER requires Clang/libFuzzer")
  endif()
  target_compile_options(robodog_ubx_rtcm_stream PRIVATE
    -fsanitize=fuzzer-no-link,address,undefined -fno-omit-frame-pointer)
  add_executable(stream_decoder_fuzz tests/fuzz_stream_decoder.cpp)
  target_link_libraries(stream_decoder_fuzz PRIVATE robodog_ubx_rtcm_stream)
  target_compile_options(stream_decoder_fuzz PRIVATE -fsanitize=fuzzer,address,undefined -fno-omit-frame-pointer)
  target_link_options(stream_decoder_fuzz PRIVATE -fsanitize=fuzzer,address,undefined)
endif()
