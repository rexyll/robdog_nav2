cmake_minimum_required(VERSION 3.16)
project(robodog_ubx_rtcm_stream LANGUAGES CXX)

option(ROBODOG_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(ROBODOG_BUILD_FUZZER "Build the libFuzzer byte-stream target (Clang only)" OFF)
option(ROBODOG_ENABLE_TSAN "Enable ThreadSanitizer" OFF)

if(ROBODOG_ENABLE_TSAN AND ROBODOG_ENABLE_SANITIZERS)
  message(FATAL_ERROR "ThreadSanitizer cannot be combined with AddressSanitizer")
endif()

add_library(robodog_ubx_rtcm_stream src/stream_decoder.cpp)
target_include_directories(robodog_ubx_rtcm_stream PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(robodog_ubx_rtcm_stream PUBLIC cxx_std_17)
target_compile_options(robodog_ubx_rtcm_stream PRIVATE -Wall -Wextra -Wpedantic -Wconversion)

if(ROBODOG_ENABLE_TSAN AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(robodog_ubx_rtcm_stream PUBLIC -fsanitize=thread -fno-omit-frame-pointer)
  target_link_options(robodog_ubx_rtcm_stream PUBLIC -fsanitize=thread)
endif()

if(ROBODOG_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(robodog_ubx_rtcm_stream PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(robodog_ubx_rtcm_stream PUBLIC -fsanitize=address,undefined)
endif()

find_package(Threads REQUIRED)

enable_testing()
add_executable(test_bounded_frame_queue tests/test_bounded_frame_queue.cpp)
target_link_libraries(test_bounded_frame_queue PRIVATE robodog_ubx_rtcm_stream Threads::Threads)
target_compile_options(test_bounded_frame_queue PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME bounded_frame_queue_tests COMMAND test_bounded_frame_queue)

add_executable(test_transfer_lifecycle tests/test_transfer_lifecycle.cpp)
target_link_libraries(test_transfer_lifecycle PRIVATE robodog_ubx_rtcm_stream Threads::Threads)
target_compile_options(test_transfer_lifecycle PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME transfer_lifecycle_tests COMMAND test_transfer_lifecycle)

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
