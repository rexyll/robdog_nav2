#include "robodog_gnss_transport/gnss_ingress_message.hpp"
#include <gtest/gtest.h>
using robodog_protocol::ChecksumStatus;
using robodog_protocol::Frame;
using robodog_protocol::GnssIngressEvent;
using robodog_protocol::GnssIngressEventType;
using robodog_protocol::Protocol;
using robodog_gnss_transport::to_ros_message;
TEST(GnssIngressMessage, ConvertsLifecycleEventWithEmptyFrameSentinels) {
  GnssIngressEvent event;
  event.type = GnssIngressEventType::GenerationStarted;
  event.process_session_id = "process-test";
  event.connection_generation = 3;
  event.ingress_sequence = 0;
  event.receive_monotonic_ns = 123456;
  const auto message = to_ros_message(event);
  EXPECT_EQ(message.event_type, message.EVENT_GENERATION_STARTED);
  EXPECT_EQ(message.process_session_id, "process-test");
  EXPECT_EQ(message.connection_generation, 3u);
  EXPECT_EQ(message.ingress_sequence, 0u);
  EXPECT_EQ(message.receive_monotonic_ns, 123456u);
  EXPECT_EQ(message.protocol, message.PROTOCOL_NONE);
  EXPECT_EQ(message.checksum_status, message.CHECKSUM_NOT_APPLICABLE);
  EXPECT_TRUE(message.frame_bytes.empty());
}
TEST(GnssIngressMessage, ConvertsFrameAndPreservesTransportMetadata) {
  GnssIngressEvent event;
  event.type = GnssIngressEventType::Frame;
  event.process_session_id = "process-test";
  event.connection_generation = 4;
  event.ingress_sequence = 8;
  event.receive_monotonic_ns = 9000;
  event.dropped_frame_events_total = 2;
  event.frame = Frame{Protocol::Ubx, {0xB5, 0x62, 0x01, 0x07}, 17, 4, ChecksumStatus::Valid};
  const auto message = to_ros_message(event);
  EXPECT_EQ(message.event_type, message.EVENT_FRAME);
  EXPECT_EQ(message.protocol, message.PROTOCOL_UBX);
  EXPECT_EQ(message.checksum_status, message.CHECKSUM_VALID);
  EXPECT_EQ(message.frame_sequence, 17u);
  EXPECT_EQ(message.frame_connection_generation, 4u);
  EXPECT_EQ(message.dropped_frame_events_total, 2u);
  ASSERT_EQ(message.frame_bytes.size(), 4u);
  EXPECT_EQ(message.frame_bytes[0], 0xB5);
}
TEST(GnssIngressMessage, RejectsFrameGenerationMismatch) {
  GnssIngressEvent event;
  event.type = GnssIngressEventType::Frame;
  event.process_session_id = "process-test";
  event.connection_generation = 4;
  event.frame = Frame{Protocol::Nmea, {'$', 'G'}, 1, 3, ChecksumStatus::Absent};
  EXPECT_THROW(to_ros_message(event), std::invalid_argument);
}
TEST(GnssIngressMessage, RejectsFramePayloadOnLifecycleEvent) {
  GnssIngressEvent event;
  event.type = GnssIngressEventType::Stopped;
  event.frame = Frame{Protocol::Ubx, {0xB5, 0x62}, 0, 0, ChecksumStatus::Valid};
  EXPECT_THROW(to_ros_message(event), std::invalid_argument);
}

TEST(GnssIngressMessage, ConvertsRtcmAndNmeaProtocolEnums) {
  GnssIngressEvent rtcm;
  rtcm.type = GnssIngressEventType::Frame;
  rtcm.process_session_id = "process-test";
  rtcm.connection_generation = 9;
  rtcm.frame = Frame{Protocol::Rtcm3, {0xD3, 0x00, 0x00}, 5, 9, ChecksumStatus::Valid};
  const auto rtcm_message = to_ros_message(rtcm);
  EXPECT_EQ(rtcm_message.protocol, rtcm_message.PROTOCOL_RTCM3);

  GnssIngressEvent nmea;
  nmea.type = GnssIngressEventType::Frame;
  nmea.process_session_id = "process-test";
  nmea.connection_generation = 9;
  nmea.frame = Frame{Protocol::Nmea, {'$', 'G', 'P'}, 6, 9, ChecksumStatus::Absent};
  const auto nmea_message = to_ros_message(nmea);
  EXPECT_EQ(nmea_message.protocol, nmea_message.PROTOCOL_NMEA);
  EXPECT_EQ(nmea_message.checksum_status, nmea_message.CHECKSUM_ABSENT);
}

TEST(GnssIngressMessage, RejectsFrameEventWithoutPayload) {
  GnssIngressEvent event;
  event.type = GnssIngressEventType::Frame;
  event.process_session_id = "process-test";
  event.connection_generation = 1;
  EXPECT_THROW(to_ros_message(event), std::invalid_argument);
}
