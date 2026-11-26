#include "robodog_gnss_transport/gnss_ingress_message.hpp"
#include "robodog_protocol/gnss_usb_stream.hpp"
#include "robodog_protocol/libusb_api_binding.hpp"
#include <rclcpp/rclcpp.hpp>
#include <libusb.h>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
namespace robodog_gnss_transport {
namespace {
std::string make_process_session_id() {
  std::random_device random;
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (int i = 0; i < 4; ++i) stream << std::setw(8) << static_cast<std::uint32_t>(random());
  return stream.str();
}
std::int64_t positive_parameter(rclcpp::Node & node, const std::string & name,
                                std::int64_t default_value, std::int64_t maximum) {
  const auto value = node.declare_parameter<std::int64_t>(name, default_value);
  if (value <= 0 || value > maximum) throw std::invalid_argument(name + " is outside its supported range");
  return value;
}
class GnssUsbTransportNode final : public rclcpp::Node {
public:
  GnssUsbTransportNode() : Node("gnss_usb_transport") {
    const auto vendor_id = positive_parameter(*this, "vendor_id", 0, 0xffff);
    const auto product_id = positive_parameter(*this, "product_id", 0, 0xffff);
    const auto queue_capacity = positive_parameter(*this, "frame_queue_capacity", 256, 65536);
    const auto history_depth = positive_parameter(*this, "dds_history_depth", 256, 65536);
    const auto read_buffer_bytes = positive_parameter(*this, "read_buffer_bytes", 12800, 16 * 1024 * 1024);
    const auto in_flight_reads = positive_parameter(*this, "in_flight_reads", 4, 64);
    const auto poll_timeout_ms = positive_parameter(*this, "event_poll_timeout_ms", 50, 1000);
    if (history_depth < queue_capacity) throw std::invalid_argument("dds_history_depth must be at least frame_queue_capacity");
    const auto topic = declare_parameter<std::string>("event_topic", "/gnss/ingress");
    const auto serial = declare_parameter<std::string>("serial_number", "");
    const auto require_nmea_checksum = declare_parameter<bool>("require_nmea_checksum", false);
    if (topic.empty()) throw std::invalid_argument("event_topic must not be empty");
    robodog_protocol::GnssUsbStreamConfig config;
    config.device.vendor_id = static_cast<std::uint16_t>(vendor_id);
    config.device.product_id = static_cast<std::uint16_t>(product_id);
    config.device.serial_number = serial;
    config.process_session_id = make_process_session_id();
    config.frame_queue_capacity = static_cast<std::size_t>(queue_capacity);
    config.read_buffer_bytes = static_cast<std::size_t>(read_buffer_bytes);
    config.in_flight_reads = static_cast<std::size_t>(in_flight_reads);
    config.event_poll_timeout_ms = static_cast<unsigned int>(poll_timeout_ms);
    config.require_nmea_checksum = require_nmea_checksum;
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(history_depth)))
        .reliable().durability_volatile();
    publisher_ = create_publisher<RosIngressEvent>(topic, qos);
    stream_ = std::make_unique<robodog_protocol::GnssUsbStream>(
        robodog_protocol::make_libusb_device_api(),
        [](void * context) {
          return robodog_protocol::make_libusb_api(static_cast<libusb_context *>(context));
        }, std::move(config));
    if (!stream_->start()) throw std::runtime_error("failed to open GNSS USB device or submit initial bulk-IN transfers");
    publisher_thread_ = std::thread([this]() { publish_events(); });
    RCLCPP_INFO(get_logger(), "GNSS USB ingress publisher started: topic=%s VID=0x%04llx PID=0x%04llx",
        topic.c_str(), static_cast<unsigned long long>(vendor_id), static_cast<unsigned long long>(product_id));
  }
  ~GnssUsbTransportNode() override {
    if (stream_ && !stream_->stop(5000)) {
      RCLCPP_FATAL(get_logger(), "GNSS USB callback drain timed out; refusing unsafe transport teardown");
    }
    stop_publisher_.store(true, std::memory_order_release);
    if (publisher_thread_.joinable()) publisher_thread_.join();
  }
private:
  void publish_events() noexcept {
    for (;;) {
      auto event = stream_->try_pop_event();
      if (event) {
        try { publisher_->publish(to_ros_message(*event)); }
        catch (const std::exception & error) {
          RCLCPP_FATAL(get_logger(), "failed to publish ordered GNSS ingress event: %s", error.what());
          rclcpp::shutdown(); return;
        } catch (...) {
          RCLCPP_FATAL(get_logger(), "unknown exception publishing GNSS ingress event");
          rclcpp::shutdown(); return;
        }
        continue;
      }
      if (stop_publisher_.load(std::memory_order_acquire)) return;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  std::unique_ptr<robodog_protocol::GnssUsbStream> stream_;
  rclcpp::Publisher<RosIngressEvent>::SharedPtr publisher_;
  std::atomic<bool> stop_publisher_{false};
  std::thread publisher_thread_;
};
}  // namespace
}  // namespace robodog_gnss_transport
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<robodog_gnss_transport::GnssUsbTransportNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("gnss_usb_transport"), "GNSS USB transport node failed to start: %s", error.what());
    result = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
