#pragma once
#include "robodog_protocol/gnss_ingress_queue.hpp"
#include "robodog_gnss_interfaces/msg/gnss_ingress_event.hpp"
#include <stdexcept>
namespace robodog_gnss_transport {
using RosIngressEvent = robodog_gnss_interfaces::msg::GnssIngressEvent;
inline RosIngressEvent to_ros_message(const robodog_protocol::GnssIngressEvent & event) {
  RosIngressEvent message;
  message.process_session_id = event.process_session_id;
  message.connection_generation = event.connection_generation;
  message.ingress_sequence = event.ingress_sequence;
  message.receive_monotonic_ns = event.receive_monotonic_ns;
  message.dropped_frame_events_total = event.dropped_frame_events_total;
  switch (event.type) {
    case robodog_protocol::GnssIngressEventType::GenerationStarted: message.event_type = RosIngressEvent::EVENT_GENERATION_STARTED; break;
    case robodog_protocol::GnssIngressEventType::Frame: message.event_type = RosIngressEvent::EVENT_FRAME; break;
    case robodog_protocol::GnssIngressEventType::DeviceLost: message.event_type = RosIngressEvent::EVENT_DEVICE_LOST; break;
    case robodog_protocol::GnssIngressEventType::TransportFault: message.event_type = RosIngressEvent::EVENT_TRANSPORT_FAULT; break;
    case robodog_protocol::GnssIngressEventType::Stopped: message.event_type = RosIngressEvent::EVENT_STOPPED; break;
    default: throw std::invalid_argument("unknown GNSS ingress event type");
  }
  message.protocol = RosIngressEvent::PROTOCOL_NONE;
  message.checksum_status = RosIngressEvent::CHECKSUM_NOT_APPLICABLE;
  message.frame_sequence = 0;
  message.frame_connection_generation = 0;
  message.frame_bytes.clear();
  if (event.type != robodog_protocol::GnssIngressEventType::Frame) {
    if (event.frame.has_value()) throw std::invalid_argument("lifecycle event must not contain a frame");
    return message;
  }
  if (!event.frame.has_value()) throw std::invalid_argument("frame event is missing its frame payload");
  const auto & frame = *event.frame;
  if (frame.connection_generation != event.connection_generation) {
    throw std::invalid_argument("frame generation does not match event generation");
  }
  switch (frame.protocol) {
    case robodog_protocol::Protocol::Ubx: message.protocol = RosIngressEvent::PROTOCOL_UBX; break;
    case robodog_protocol::Protocol::Rtcm3: message.protocol = RosIngressEvent::PROTOCOL_RTCM3; break;
    case robodog_protocol::Protocol::Nmea: message.protocol = RosIngressEvent::PROTOCOL_NMEA; break;
    default: throw std::invalid_argument("unknown GNSS frame protocol");
  }
  switch (frame.checksum_status) {
    case robodog_protocol::ChecksumStatus::NotApplicable: message.checksum_status = RosIngressEvent::CHECKSUM_NOT_APPLICABLE; break;
    case robodog_protocol::ChecksumStatus::Valid: message.checksum_status = RosIngressEvent::CHECKSUM_VALID; break;
    case robodog_protocol::ChecksumStatus::Absent: message.checksum_status = RosIngressEvent::CHECKSUM_ABSENT; break;
    default: throw std::invalid_argument("unknown GNSS frame checksum status");
  }
  message.frame_sequence = frame.sequence;
  message.frame_connection_generation = frame.connection_generation;
  message.frame_bytes = frame.bytes;
  return message;
}
}  // namespace robodog_gnss_transport
