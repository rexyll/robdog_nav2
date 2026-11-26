cmake_minimum_required(VERSION 3.16)
project(robodog_gnss_transport LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robodog_gnss_interfaces REQUIRED)
find_package(Threads REQUIRED)

# Build the transport-independent GNSS core from its single source location.
# Standalone sanitizer/fuzzer jobs continue to own its CTest targets.
set(ROBODOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ROBODOG_BUILD_FUZZER OFF CACHE BOOL "" FORCE)
set(ROBODOG_BUILD_LIBUSB_BINDING ON CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../navigation/ubx_rtcm_stream
                 ${CMAKE_CURRENT_BINARY_DIR}/ubx_rtcm_stream)

add_executable(gnss_usb_transport_node src/gnss_usb_transport_node.cpp)
target_compile_features(gnss_usb_transport_node PRIVATE cxx_std_17)
target_compile_options(gnss_usb_transport_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(gnss_usb_transport_node PRIVATE include)
target_link_libraries(gnss_usb_transport_node
  robodog_ubx_rtcm_stream
  robodog_libusb_binding
  Threads::Threads)
ament_target_dependencies(gnss_usb_transport_node
  rclcpp
  robodog_gnss_interfaces)

install(TARGETS gnss_usb_transport_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gnss_ingress_message
    test/test_gnss_ingress_message.cpp)
  target_compile_features(test_gnss_ingress_message PRIVATE cxx_std_17)
  target_include_directories(test_gnss_ingress_message PRIVATE include)
  target_link_libraries(test_gnss_ingress_message
    robodog_ubx_rtcm_stream
    Threads::Threads)
  ament_target_dependencies(test_gnss_ingress_message
    robodog_gnss_interfaces)

  ament_add_gtest(test_gnss_ingress_ros_delivery
    test/test_gnss_ingress_ros_delivery.cpp)
  target_include_directories(test_gnss_ingress_ros_delivery PRIVATE include)
  target_compile_features(test_gnss_ingress_ros_delivery PRIVATE cxx_std_17)
  ament_target_dependencies(test_gnss_ingress_ros_delivery
    rclcpp
    robodog_gnss_interfaces)
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp robodog_gnss_interfaces)
ament_package()
