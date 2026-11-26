cmake_minimum_required(VERSION 3.8)
project(robodog_gnss_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GnssIngressEvent.msg"
  "msg/GnssReceiverTime.msg"
  "msg/GnssNavPvt.msg"
  "msg/GnssNavHpPosLlh.msg"
  "msg/GnssNavCov.msg"
  "msg/GnssNavStatus.msg"
  "msg/GnssObservation.msg"
  "msg/GnssQualityStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
