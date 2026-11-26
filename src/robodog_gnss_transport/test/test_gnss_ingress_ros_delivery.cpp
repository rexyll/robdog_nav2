#include "robodog_gnss_interfaces/msg/gnss_ingress_event.hpp"
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

TEST(GnssIngressRosDelivery, ReliablePublisherThreadPreservesEventOrder) {
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  const std::string topic = "/test/gnss_ingress_ordered_delivery";
  auto publisher_node = std::make_shared<rclcpp::Node>("gnss_ingress_test_publisher");
  auto subscriber_node = std::make_shared<rclcpp::Node>("gnss_ingress_test_subscriber");
  using Message = robodog_gnss_interfaces::msg::GnssIngressEvent;
  auto publisher = publisher_node->create_publisher<Message>(
      topic, rclcpp::QoS(rclcpp::KeepLast(16)).reliable().durability_volatile());

  std::mutex received_mutex;
  std::vector<std::uint64_t> received;
  auto subscription = subscriber_node->create_subscription<Message>(
      topic, rclcpp::QoS(rclcpp::KeepLast(16)).reliable(),
      [&received_mutex, &received](Message::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(received_mutex);
        received.push_back(message->ingress_sequence);
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

  std::thread publisher_thread([publisher]() {
    for (std::uint64_t sequence = 0; sequence < 8; ++sequence) {
      Message message;
      message.event_type = Message::EVENT_FRAME;
      message.process_session_id = "process-test";
      message.connection_generation = 1;
      message.ingress_sequence = sequence;
      message.receive_monotonic_ns = 1000 + sequence;
      message.protocol = Message::PROTOCOL_UBX;
      message.checksum_status = Message::CHECKSUM_VALID;
      message.frame_sequence = sequence;
      message.frame_connection_generation = 1;
      message.frame_bytes = {0xB5, 0x62};
      publisher->publish(message);
    }
  });

  const auto receive_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < receive_deadline) {
    executor.spin_some();
    {
      std::lock_guard<std::mutex> lock(received_mutex);
      if (received.size() == 8) break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  publisher_thread.join();

  {
    std::lock_guard<std::mutex> lock(received_mutex);
    ASSERT_EQ(received.size(), 8u);
    for (std::size_t i = 0; i < received.size(); ++i) {
      EXPECT_EQ(received[i], i);
    }
  }

  executor.remove_node(subscriber_node);
  subscription.reset();
  publisher.reset();
  subscriber_node.reset();
  publisher_node.reset();
  rclcpp::shutdown();
}
