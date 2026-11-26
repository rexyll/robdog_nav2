#pragma once

// A libusb-shaped adapter seam with no compile-time dependency on libusb headers.
// Production code supplies wrappers around libusb-1.0; tests inject deterministic functions.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <exception>
#include <thread>
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

enum class LibusbCancelResult {
  Requested,
  NotPending,
  NotFound,
  DeviceLost,
  Failed
};

struct LibusbCancelRequestResult {
  std::uint64_t transfer_id{};
  LibusbCancelResult result{LibusbCancelResult::Failed};
};

enum class UsbTransferDirection { In, Out };

struct LibusbTransferRequest {
  std::uint8_t endpoint{};
  UsbTransferDirection direction{UsbTransferDirection::In};
  std::vector<std::uint8_t> buffer;
  unsigned int timeout_ms{};
  std::uint8_t transfer_flags{};
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
      int, unsigned int, std::uint8_t, NativeCallback, void *)> fill_bulk_transfer;
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
    int cancel_not_found_code{-5};
    int cancel_no_device_code{-4};
    // false when a higher-level DeviceSession owns the native device handle.
    bool close_device_on_close{true};
  };

  LibusbTransferTransport(Api api, void * device_handle)
  : api_(std::move(api)), device_handle_(device_handle) {}

  LibusbTransferTransport(const LibusbTransferTransport &) = delete;
  LibusbTransferTransport & operator=(const LibusbTransferTransport &) = delete;

  ~LibusbTransferTransport() { if (!close()) std::terminate(); }

  bool valid() const {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    return api_valid();
  }

  bool submit(
    std::uint64_t id, std::uint64_t generation,
    LibusbTransferRequest request, Completion completion)
  {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    const bool endpoint_is_in = (request.endpoint & 0x80u) != 0;
    if (!api_valid() || closed_ || !accepting_submissions_ || !completion ||
        endpoint_is_in != (request.direction == UsbTransferDirection::In) ||
        (request.endpoint & 0x70u) != 0 || (request.endpoint & 0x0fu) == 0 || request.buffer.size() >
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
      request.timeout_ms, request.transfer_flags,
      &LibusbTransferTransport::callback_trampoline, raw);
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

  LibusbCancelResult cancel_detailed(std::uint64_t id) {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    return cancel_locked(id);
  }

  std::vector<LibusbCancelRequestResult> cancel_all_detailed() {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    std::vector<std::uint64_t> ids;
    {
      std::lock_guard<std::mutex> registry_lock(registry_mutex_);
      ids.reserve(pending_.size());
      for (const auto & entry : pending_) ids.push_back(entry.first);
    }
    std::vector<LibusbCancelRequestResult> results;
    results.reserve(ids.size());
    for (const auto id : ids) results.push_back({id, cancel_locked(id)});
    return results;
  }

  void stop_submissions() {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    accepting_submissions_ = false;
  }

  bool drained() const {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (callbacks_in_flight_ != 0) return false;
    std::lock_guard<std::mutex> registry_lock(registry_mutex_);
    return pending_.empty();
  }

  bool bind_event_thread() {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (closed_ || !api_valid()) return false;
    if (event_thread_bound_) return event_thread_id_ == std::this_thread::get_id();
    event_thread_id_ = std::this_thread::get_id();
    event_thread_bound_ = true;
    return true;
  }

  bool unbind_event_thread() {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (!event_thread_bound_ || event_thread_id_ != std::this_thread::get_id() ||
        callbacks_in_flight_ != 0) return false;
    {
      std::lock_guard<std::mutex> registry_lock(registry_mutex_);
      if (!pending_.empty()) return false;
    }
    event_thread_id_ = std::thread::id{};
    event_thread_bound_ = false;
    return true;
  }

  bool cancel(std::uint64_t id) {
    return cancel_detailed(id) == LibusbCancelResult::Requested;
  }

  int poll_events(unsigned int timeout_ms = 0) {
    std::vector<std::unique_ptr<Context>> drained;
    int rc = -1;
    {
      std::lock_guard<std::mutex> operation_lock(operation_mutex_);
      if (closed_ || !api_valid() ||
          (event_thread_bound_ && event_thread_id_ != std::this_thread::get_id())) return -1;
      rc = api_.handle_events(timeout_ms);
      drained = take_callbacks_after_event_pump();
      for (auto & context : drained) {
        // libusb actual_length is untrusted transport metadata. Never expose a
        // byte count larger than the submitted buffer, and reject negative
        // values rather than silently normalizing them to an empty success.
        const bool invalid_actual_length =
          context->actual_length < 0 ||
          static_cast<std::size_t>(context->actual_length) > context->buffer.size();
        if (invalid_actual_length) {
          context->native_status = api_.overflow_status;
          context->actual_length = 0;
          context->buffer.clear();
        } else {
          context->buffer.resize(static_cast<std::size_t>(context->actual_length));
        }
        api_.free_transfer(context->transfer);
        context->transfer = nullptr;
      }
      callbacks_in_flight_ += drained.size();
    }

    // User callbacks run outside operation_mutex_: they may submit new work.
    std::exception_ptr callback_error;
    for (auto & context : drained) {
      try {
        context->completion(
          context->id, context->generation, map_status(context->native_status),
          std::move(context->buffer), context->actual_length);
      } catch (...) {
        if (!callback_error) callback_error = std::current_exception();
      }
      std::lock_guard<std::mutex> operation_lock(operation_mutex_);
      --callbacks_in_flight_;
    }
    if (callback_error) std::rethrow_exception(callback_error);
    return rc;
  }

  bool close() {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (closed_) return true;
    if (event_thread_bound_ || callbacks_in_flight_ != 0) return false;
    {
      std::lock_guard<std::mutex> lock(registry_mutex_);
      if (!pending_.empty()) return false;
    }
    if (api_.close_device_on_close && api_.close_device && device_handle_) api_.close_device(device_handle_);
    device_handle_ = nullptr;
    closed_ = true;
    return true;
  }

  std::size_t pending_count() const {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    return pending_.size();
  }

private:
  LibusbCancelResult cancel_locked(std::uint64_t id) {
    if (closed_) return LibusbCancelResult::Failed;
    NativeTransfer transfer = nullptr;
    {
      // Keep the registry critical section separate from the native API call.
      // operation_mutex_ prevents poll/close from invalidating this pointer.
      std::lock_guard<std::mutex> registry_lock(registry_mutex_);
      auto it = pending_.find(id);
      if (it == pending_.end() || it->second->callback_seen) return LibusbCancelResult::NotPending;
      transfer = it->second->transfer;
    }
    const int rc = api_.cancel_transfer(transfer);
    if (rc == api_.success_code) return LibusbCancelResult::Requested;
    if (rc == api_.cancel_not_found_code) return LibusbCancelResult::NotFound;
    if (rc == api_.cancel_no_device_code) return LibusbCancelResult::DeviceLost;
    return LibusbCancelResult::Failed;
  }

  bool api_valid() const {
    return api_.alloc_transfer && api_.free_transfer && api_.fill_bulk_transfer &&
           api_.submit_transfer && api_.cancel_transfer && api_.handle_events &&
           api_.close_device && device_handle_ != nullptr;
  }

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

  std::vector<std::unique_ptr<Context>> take_callbacks_after_event_pump() {
    std::vector<std::unique_ptr<Context>> drained;
    std::lock_guard<std::mutex> lock(registry_mutex_);
    for (auto it = pending_.begin(); it != pending_.end();) {
      if (it->second->callback_seen) {
        drained.push_back(std::move(it->second));
        it = pending_.erase(it);
      } else {
        ++it;
      }
    }
    return drained;
  }

  Api api_;
  void * device_handle_{};
  mutable std::mutex registry_mutex_;
  mutable std::mutex operation_mutex_;
  std::map<std::uint64_t, std::unique_ptr<Context>> pending_;
  bool closed_{false};
  bool accepting_submissions_{true};
  bool event_thread_bound_{false};
  std::thread::id event_thread_id_{};
  std::size_t callbacks_in_flight_{0};
};

}  // namespace robodog_protocol
