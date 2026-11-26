#include "robodog_protocol/libusb_event_loop.hpp"

#include <chrono>
#include <exception>
#include <stdexcept>

namespace robodog_protocol {

LibusbEventLoop::LibusbEventLoop(
  LibusbTransferTransport & transport, unsigned int poll_timeout_ms)
: transport_(transport), poll_timeout_ms_(poll_timeout_ms) {}

LibusbEventLoop::~LibusbEventLoop()
{
  // Destruction without a successful shutdown would leave a thread referencing
  // this object and potentially pending native transfers. Fail closed.
  if (worker_.joinable()) std::terminate();
}

bool LibusbEventLoop::start()
{
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  std::unique_lock<std::mutex> lock(mutex_);
  if (started_ || worker_.joinable()) return false;
  stopping_.store(false, std::memory_order_release);
  last_event_error_ = 0;
  callback_exception_seen_ = false;
  started_ = true;
  thread_ready_ = false;
  thread_start_failed_ = false;
  worker_ = std::thread(&LibusbEventLoop::run, this);
  const bool ready = condition_.wait_for(lock, std::chrono::seconds(2), [this] {
    return thread_ready_ || thread_start_failed_;
  });
  if (ready && thread_ready_) return true;
  stopping_.store(true, std::memory_order_release);
  lock.unlock();
  if (worker_.joinable()) worker_.join();
  lock.lock();
  started_ = false;
  return false;
}

bool LibusbEventLoop::shutdown(std::uint32_t timeout_ms)
{
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  bool join_failed_startup = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (worker_.joinable() && worker_.get_id() == std::this_thread::get_id()) return false;
    if (!started_) {
      if (worker_.joinable()) join_failed_startup = true;
      else return transport_.drained();
    }
  }
  if (join_failed_startup) {
    worker_.join();
    return transport_.drained();
  }

  transport_.stop_submissions();
  (void)transport_.cancel_all_detailed();

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);
  while (!transport_.drained()) {
    if (std::chrono::steady_clock::now() >= deadline) return false;
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait_for(lock, std::chrono::milliseconds(5));
  }

  stopping_.store(true, std::memory_order_release);
  condition_.notify_all();
  if (worker_.joinable()) worker_.join();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    started_ = false;
  }
  return true;
}

int LibusbEventLoop::last_event_error() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_event_error_;
}

bool LibusbEventLoop::callback_exception_seen() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return callback_exception_seen_;
}

void LibusbEventLoop::run()
{
  if (!transport_.bind_event_thread()) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_event_error_ = -1;
    thread_start_failed_ = true;
    started_ = false;
    condition_.notify_all();
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    thread_ready_ = true;
    condition_.notify_all();
  }

  while (!stopping_.load(std::memory_order_acquire)) {
    int rc = -1;
    try {
      rc = transport_.poll_events(poll_timeout_ms_);
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      callback_exception_seen_ = true;
    }

    if (rc < 0) {
      std::lock_guard<std::mutex> lock(mutex_);
      last_event_error_ = rc;
    }
    condition_.notify_all();
    if (rc < 0) std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  if (!transport_.unbind_event_thread()) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_event_error_ = -2;
  }
  condition_.notify_all();
}

}  // namespace robodog_protocol
