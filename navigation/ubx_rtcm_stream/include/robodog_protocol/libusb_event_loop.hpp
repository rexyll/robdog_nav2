#pragma once

#include "robodog_protocol/libusb_transfer_transport.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

namespace robodog_protocol {

// Owns the sole event-pump thread for one LibusbTransferTransport. Call
// shutdown() before destroying the loop, transport, or its DeviceSession.
class LibusbEventLoop {
public:
  explicit LibusbEventLoop(LibusbTransferTransport & transport, unsigned int poll_timeout_ms = 50);
  LibusbEventLoop(const LibusbEventLoop &) = delete;
  LibusbEventLoop & operator=(const LibusbEventLoop &) = delete;
  ~LibusbEventLoop();

  bool start();
  // Stops new submissions, requests cancellation of all pending transfers,
  // pumps callbacks until drained, then joins the event thread. On timeout,
  // the event thread remains alive so the caller can diagnose/retry.
  bool shutdown(std::uint32_t timeout_ms = 5000);

  int last_event_error() const noexcept;
  bool callback_exception_seen() const noexcept;

private:
  void run();

  LibusbTransferTransport & transport_;
  const unsigned int poll_timeout_ms_;
  std::atomic<bool> stopping_{false};
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::thread worker_;
  bool started_{false};
  int last_event_error_{0};
  bool callback_exception_seen_{false};
};

}  // namespace robodog_protocol
