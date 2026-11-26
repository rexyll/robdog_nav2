cmake_minimum_required(VERSION 3.16)
project(robodog_ubx_rtcm_stream LANGUAGES CXX)

option(ROBODOG_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(ROBODOG_BUILD_FUZZER "Build the libFuzzer byte-stream target (Clang only)" OFF)
option(ROBODOG_ENABLE_TSAN "Enable ThreadSanitizer" OFF)
option(ROBODOG_BUILD_LIBUSB_BINDING "Build the production libusb-1.0 function-table binding" OFF)

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

if(ROBODOG_BUILD_LIBUSB_BINDING)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)
  add_library(robodog_libusb_binding src/libusb_api_binding.cpp)
  target_include_directories(robodog_libusb_binding PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(robodog_libusb_binding PUBLIC cxx_std_17)
  target_compile_options(robodog_libusb_binding PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(robodog_libusb_binding PUBLIC
    robodog_ubx_rtcm_stream PkgConfig::LIBUSB)
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

add_executable(test_fake_usb_transport tests/test_fake_usb_transport.cpp)
target_link_libraries(test_fake_usb_transport PRIVATE robodog_ubx_rtcm_stream Threads::Threads)
target_compile_options(test_fake_usb_transport PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME fake_usb_transport_tests COMMAND test_fake_usb_transport)

add_executable(test_libusb_transfer_transport tests/test_libusb_transfer_transport.cpp)
target_link_libraries(test_libusb_transfer_transport PRIVATE robodog_ubx_rtcm_stream Threads::Threads)
target_compile_options(test_libusb_transfer_transport PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME libusb_transfer_transport_tests COMMAND test_libusb_transfer_transport)

if(ROBODOG_BUILD_LIBUSB_BINDING)
  add_executable(test_libusb_api_binding tests/test_libusb_api_binding.cpp)
  target_link_libraries(test_libusb_api_binding PRIVATE robodog_libusb_binding)
  target_compile_options(test_libusb_api_binding PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME libusb_api_binding_tests COMMAND test_libusb_api_binding)
endif()

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
