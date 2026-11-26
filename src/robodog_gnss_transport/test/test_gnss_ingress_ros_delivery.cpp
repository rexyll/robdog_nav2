#include "robodog_gnss_transport/gnss_ingress_publisher_pump.hpp"
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

using robodog_protocol::ChecksumStatus;
using robodog_protocol::Frame;
using robodog_protocol::GnssIngressEvent;
using robodog_protocol::GnssIngressEventType;
using robodog_protocol::Protocol;
using robodog_gnss_transport::GnssIngressPublisherPump;

static std::deque<GnssIngressEvent> make_events() {
  std::deque<GnssIngressEvent> events;
  GnssIngressEvent started;
  started.type = GnssIngressEventType::GenerationStarted;
  started.process_session_id = "process-pump-test";
  started.connection_generation = 1;
  started.ingress_sequence = 0;
  started.receive_monotonic_ns = 100;
  events.push_back(started);

  GnssIngressEvent frame;
  frame.type = GnssIngressEventType::Frame;
  frame.process_session_id = "process-pump-test";
  frame.connection_generation = 1;
  frame.ingress_sequence = 1;
  frame.receive_monotonic_ns = 110;
  frame.frame = Frame{Protocol::Ubx, {0xB5, 0x62}, 0, 1, ChecksumStatus::Valid};
  events.push_back(frame);

  GnssIngressEvent stopped;
  stopped.type = GnssIngressEventType::Stopped;
  stopped.process_session_id = "process-pump-test";
  stopped.connection_generation = 1;
  stopped.ingress_sequence = 2;
  stopped.receive_monotonic_ns = 120;
  events.push_back(stopped);
  return events;
}

TEST(GnssIngressRosDelivery, MatchGateAndReliablePumpPreserveEventOrder) {
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  using Message = robodog_gnss_interfaces::msg::GnssIngressEvent;
  const std::string topic = "/test/gnss_ingress_pump_order";
  auto publisher_node = std::make_shared<rclcpp::Node>("gnss_ingress_pump_publisher");
  auto subscriber_node = std::make_shared<rclcpp::Node>("gnss_ingress_pump_subscriber");
  auto publisher = publisher_node->create_publisher<Message>(
      topic, rclcpp::QoS(rclcpp::KeepLast(16)).reliable().durability_volatile());

  std::mutex source_mutex;
  auto source = make_events();
  std::atomic<std::size_t> pop_calls{0};
  auto pop_event = [&]() -> std::optional<GnssIngressEvent> {
    std::lock_guard<std::mutex> lock(source_mutex);
    ++pop_calls;
    if (source.empty()) return std::nullopt;
    auto event = std::move(source.front());
    source.pop_front();
    return event;
  };
  std::atomic<bool> worker_error{false};
  GnssIngressPublisherPump pump(
      pop_event,
      [publisher](const Message & message) { publisher->publish(message); },
      [publisher]() { return publisher->get_subscription_count(); },
      [&worker_error](std::exception_ptr) { worker_error.store(true); });

  ASSERT_TRUE(pump.start());
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(pop_calls.load(), 0u);  // no subscriber: retain GenerationStarted
  {
    std::lock_guard<std::mutex> lock(source_mutex);
    EXPECT_EQ(source.size(), 3u);
  }

  std::mutex received_mutex;
  std::vector<std::uint64_t> received_sequences;
  std::vector<std::uint8_t> received_types;
  auto subscription = subscriber_node->create_subscription<Message>(
      topic, rclcpp::QoS(rclcpp::KeepLast(16)).reliable(),
      [&received_mutex, &received_sequences, &received_types](Message::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(received_mutex);
        received_sequences.push_back(message->ingress_sequence);
        received_types.push_back(message->event_type);
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(subscriber_node);
  const auto match_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (publisher->get_subscription_count() == 0 &&
         std::chrono::steady_clock::now() < match_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GT(publisher->get_subscription_count(), 0u);

  const auto receive_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < receive_deadline) {
    executor.spin_some();
    {
      std::lock_guard<std::mutex> lock(received_mutex);
      if (received_sequences.size() == 3) break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  pump.stop_and_join();
  EXPECT_FALSE(pump.failed());
  EXPECT_FALSE(worker_error.load());
  {
    std::lock_guard<std::mutex> lock(received_mutex);
    ASSERT_EQ(received_sequences.size(), 3u);
    EXPECT_EQ(received_sequences[0], 0u);
    EXPECT_EQ(received_sequences[1], 1u);
    EXPECT_EQ(received_sequences[2], 2u);
    EXPECT_EQ(received_types[0], Message::EVENT_GENERATION_STARTED);
    EXPECT_EQ(received_types[1], Message::EVENT_FRAME);
    EXPECT_EQ(received_types[2], Message::EVENT_STOPPED);
  }
  executor.remove_node(subscriber_node);
  subscription.reset();
  publisher.reset();
  subscriber_node.reset();
  publisher_node.reset();
  rclcpp::shutdown();
}

TEST(GnssIngressPublisherPump, ShutdownDrainsWithoutMatchedSubscriber) {
  std::mutex source_mutex;
  auto source = make_events();
  std::vector<std::uint64_t> published;
  std::atomic<std::size_t> pop_calls{0};
  GnssIngressPublisherPump pump(
      [&]() -> std::optional<GnssIngressEvent> {
        std::lock_guard<std::mutex> lock(source_mutex);
        ++pop_calls;
        if (source.empty()) return std::nullopt;
        auto event = std::move(source.front());
        source.pop_front();
        return event;
      },
      [&](const robodog_gnss_transport::RosIngressEvent & message) {
        published.push_back(message.ingress_sequence);
      },
      []() { return std::size_t{0}; });
  ASSERT_TRUE(pump.start());
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(pop_calls.load(), 0u);
  pump.stop_and_join();
  EXPECT_FALSE(pump.failed());
  EXPECT_EQ(published, (std::vector<std::uint64_t>{0, 1, 2}));
  EXPECT_TRUE(source.empty());
}
