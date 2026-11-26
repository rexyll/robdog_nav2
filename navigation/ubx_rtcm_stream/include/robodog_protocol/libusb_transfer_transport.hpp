#pragma once

// A libusb-shaped adapter seam with no compile-time dependency on libusb headers.
// Production code supplies wrappers around libusb-1.0; tests inject deterministic functions.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace robodog_protocol {

enum class LibusbCompletionStatus {
  Completed,
  Error,
  TimedOut,
  Cancelled,
  Stall,
  NoDevice,
  Overflow,
  Unknown
};

enum class UsbTransferDirection { In, Out };

struct LibusbTransferRequest {
  std::uint8_t endpoint{};
  UsbTransferDirection direction{UsbTransferDirection::In};
  std::vector<std::uint8_t> buffer;
  unsigned int timeout_ms{};
};

class LibusbTransferTransport {
public:
  using NativeTransfer = void *;
  using NativeCallback = void (*)(NativeTransfer, int, int, void *);
  using Completion = std::function<void(
    std::uint64_t, std::uint64_t, LibusbCompletionStatus,
    std::vector<std::uint8_t>, int)>;

  struct Api {
    std::function<NativeTransfer(int)> alloc_transfer;
    std::function<void(NativeTransfer)> free_transfer;
    std::function<int(
      NativeTransfer, void *, std::uint8_t, std::uint8_t *,
      int, unsigned int, NativeCallback, void *)> fill_bulk_transfer;
    std::function<int(NativeTransfer)> submit_transfer;
    std::function<int(NativeTransfer)> cancel_transfer;
    std::function<int(unsigned int)> handle_events;
    std::function<void(void *)> close_device;

    int success_code{0};
    int completed_status{0};
    int error_status{1};
    int timeout_status{2};
    int cancelled_status{3};
    int stall_status{4};
    int no_device_status{5};
    int overflow_status{6};
  };

  LibusbTransferTransport(Api api, void * device_handle)
  : api_(std::move(api)), device_handle_(device_handle) {}

  LibusbTransferTransport(const LibusbTransferTransport &) = delete;
  LibusbTransferTransport & operator=(const LibusbTransferTransport &) = delete;

  ~LibusbTransferTransport() { if (!close()) std::terminate(); }

  bool valid() const {
    return api_.alloc_transfer && api_.free_transfer && api_.fill_bulk_transfer &&
           api_.submit_transfer && api_.cancel_transfer && api_.handle_events &&
           api_.close_device && device_handle_ != nullptr;
  }

  bool submit(
    std::uint64_t id, std::uint64_t generation,
    LibusbTransferRequest request, Completion completion)
  {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    const bool endpoint_is_in = (request.endpoint & 0x80u) != 0;
    if (!valid() || closed_ || !completion ||
        endpoint_is_in != (request.direction == UsbTransferDirection::In) ||
        (request.endpoint & 0x70u) != 0 || request.buffer.size() >
        static_cast<std::size_t>(0x7fffffff)) return false;

    {
      std::lock_guard<std::mutex> lock(registry_mutex_);
      if (pending_.count(id) != 0) return false;
    }

    auto context = std::make_unique<Context>();
    context->id = id;
    context->generation = generation;
    context->buffer = std::move(request.buffer);
    context->completion = std::move(completion);
    context->transfer = api_.alloc_transfer(0);
    if (!context->transfer) return false;

    Context * raw = context.get();
    {
      // Register ownership before submit. The event pump is serialized by
      // operation_mutex_, so a callback cannot run before submit returns.
      std::lock_guard<std::mutex> lock(registry_mutex_);
      if (pending_.count(id) != 0) {
        api_.free_transfer(context->transfer);
        return false;
      }
      pending_.emplace(id, std::move(context));
    }

    const int length = static_cast<int>(raw->buffer.size());
    const int fill_rc = api_.fill_bulk_transfer(
      raw->transfer, device_handle_, request.endpoint, raw->buffer.data(), length,
      request.timeout_ms, &LibusbTransferTransport::callback_trampoline, raw);
    if (fill_rc != api_.success_code) {
      release_unsubmitted(id);
      return false;
    }

    const int submit_rc = api_.submit_transfer(raw->transfer);
    if (submit_rc != api_.success_code) {
      // A synchronous submit failure means libusb did not take ownership and
      // no completion callback is expected. This is the only post-registration
      // path that frees without callback drain.
      release_unsubmitted(id);
      return false;
    }
    return true;
  }

  bool cancel(std::uint64_t id) {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (closed_) return false;
    auto it = pending_.find(id);
    if (it == pending_.end() || it->second->callback_seen) return false;
    // A successful cancel is only a request. Ownership remains pending until
    // the callback is dispatched by handle_events().
    return api_.cancel_transfer(it->second->transfer) == api_.success_code;
  }

  int poll_events(unsigned int timeout_ms = 0) {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (closed_ || !valid()) return -1;
    const int rc = api_.handle_events(timeout_ms);
    reap_callbacks_after_event_pump();
    return rc;
  }

  bool close() {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (closed_) return true;
    {
      std::lock_guard<std::mutex> lock(registry_mutex_);
      if (!pending_.empty()) return false;
    }
    if (api_.close_device && device_handle_) api_.close_device(device_handle_);
    device_handle_ = nullptr;
    closed_ = true;
    return true;
  }

  std::size_t pending_count() const {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    return pending_.size();
  }

private:
  struct Context {
    std::uint64_t id{};
    std::uint64_t generation{};
    NativeTransfer transfer{};
    std::vector<std::uint8_t> buffer;
    Completion completion;
    bool callback_seen{false};
    int native_status{};
    int actual_length{};
  };

  static void callback_trampoline(
    NativeTransfer transfer, int status, int actual_length, void * user_data)
  {
    auto * context = static_cast<Context *>(user_data);
    if (!context || context->transfer != transfer || context->callback_seen) return;
    context->native_status = status;
    context->actual_length = actual_length;
    context->callback_seen = true;
  }

  LibusbCompletionStatus map_status(int status) const {
    if (status == api_.completed_status) return LibusbCompletionStatus::Completed;
    if (status == api_.error_status) return LibusbCompletionStatus::Error;
    if (status == api_.timeout_status) return LibusbCompletionStatus::TimedOut;
    if (status == api_.cancelled_status) return LibusbCompletionStatus::Cancelled;
    if (status == api_.stall_status) return LibusbCompletionStatus::Stall;
    if (status == api_.no_device_status) return LibusbCompletionStatus::NoDevice;
    if (status == api_.overflow_status) return LibusbCompletionStatus::Overflow;
    return LibusbCompletionStatus::Unknown;
  }

  void release_unsubmitted(std::uint64_t id) {
    std::unique_ptr<Context> context;
    {
      std::lock_guard<std::mutex> lock(registry_mutex_);
      auto it = pending_.find(id);
      if (it == pending_.end()) return;
      context = std::move(it->second);
      pending_.erase(it);
    }
    if (context->transfer) api_.free_transfer(context->transfer);
  }

  void reap_callbacks_after_event_pump() {
    std::vector<std::unique_ptr<Context>> drained;
    {
      std::lock_guard<std::mutex> lock(registry_mutex_);
      for (auto it = pending_.begin(); it != pending_.end();) {
        if (it->second->callback_seen) {
          drained.push_back(std::move(it->second));
          it = pending_.erase(it);
        } else {
          ++it;
        }
      }
    }

    // Do not free native transfers or callback contexts inside libusb's
    // callback. handle_events() has returned before this finalization runs.
    for (auto & context : drained) {
      const auto status = map_status(context->native_status);
      const int actual = std::max(0, context->actual_length);
      if (context->buffer.size() > static_cast<std::size_t>(actual)) {
        context->buffer.resize(static_cast<std::size_t>(actual));
      }
      api_.free_transfer(context->transfer);
      context->transfer = nullptr;
      context->completion(
        context->id, context->generation, status,
        std::move(context->buffer), context->actual_length);
    }
  }

  Api api_;
  void * device_handle_{};
  mutable std::mutex registry_mutex_;
  std::mutex operation_mutex_;
  std::map<std::uint64_t, std::unique_ptr<Context>> pending_;
  bool closed_{false};
};

}  // namespace robodog_protocol
