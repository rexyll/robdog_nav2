#pragma once

#include "robodog_gnss_transport/gnss_ingress_message.hpp"

#include <atomic>
#include <cstddef>
#include <chrono>
#include <exception>
#include <functional>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

namespace robodog_gnss_transport {

class GnssIngressPublisherPump {
public:
  using PopEvent = std::function<std::optional<robodog_protocol::GnssIngressEvent>()>;
  using Publish = std::function<void(const RosIngressEvent &)>;
  using SubscriberCount = std::function<std::size_t()>;
  using ErrorHandler = std::function<void(std::exception_ptr)>;

  GnssIngressPublisherPump(PopEvent pop_event, Publish publish,
                           SubscriberCount subscriber_count,
                           ErrorHandler error_handler = {})
      : pop_event_(std::move(pop_event)), publish_(std::move(publish)),
        subscriber_count_(std::move(subscriber_count)),
        error_handler_(std::move(error_handler)) {
    if (!pop_event_ || !publish_ || !subscriber_count_) {
      throw std::invalid_argument("GNSS ingress publisher pump callbacks must be set");
    }
  }

  GnssIngressPublisherPump(const GnssIngressPublisherPump &) = delete;
  GnssIngressPublisherPump & operator=(const GnssIngressPublisherPump &) = delete;
  ~GnssIngressPublisherPump() { stop_and_join(); }

  bool start() {
    if (started_ || worker_.joinable()) return false;
    stopping_.store(false, std::memory_order_release);
    worker_ = std::thread([this]() { run(); });
    started_ = true;
    return true;
  }

  // Call after the source stops accepting events; then drain even without a subscriber.
  void stop_and_join() noexcept {
    stopping_.store(true, std::memory_order_release);
    if (worker_.joinable()) worker_.join();
  }

  bool failed() const noexcept { return failed_.load(std::memory_order_acquire); }

private:
  void run() noexcept {
    try {
      for (;;) {
        if (!stopping_.load(std::memory_order_acquire) && subscriber_count_() == 0) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        }
        auto event = pop_event_();
        if (event) {
          publish_(to_ros_message(*event));
          continue;
        }
        if (stopping_.load(std::memory_order_acquire)) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    } catch (...) {
      failed_.store(true, std::memory_order_release);
      if (error_handler_) {
        try { error_handler_(std::current_exception()); } catch (...) {}
      }
    }
  }

  PopEvent pop_event_;
  Publish publish_;
  SubscriberCount subscriber_count_;
  ErrorHandler error_handler_;
  std::atomic<bool> stopping_{false};
  std::atomic<bool> failed_{false};
  bool started_{false};
  std::thread worker_;
};

}  // namespace robodog_gnss_transport
