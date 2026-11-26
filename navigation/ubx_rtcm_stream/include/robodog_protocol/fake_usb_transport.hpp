#pragma once

#include "robodog_protocol/transfer_lifecycle.hpp"

#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <utility>
#include <vector>

namespace robodog_protocol {

enum class UsbCompletionStatus { Completed, Cancelled, NoDevice, Error };

class IUsbTransferTransport {
public:
  using Completion = std::function<void(std::uint64_t, std::uint64_t, UsbCompletionStatus)>;
  virtual ~IUsbTransferTransport() = default;
  virtual bool submit(std::uint64_t id, std::uint64_t generation, Completion completion) = 0;
  virtual bool cancel(std::uint64_t id) = 0;
  virtual std::size_t poll_events() = 0;
  virtual bool close() = 0;
};

// Deterministic event-driven fake. Callbacks are dispatched only by poll_events(),
// and never while the fake's internal mutex is held.
class FakeUsbTransport final : public IUsbTransferTransport {
public:
  bool submit(std::uint64_t id, std::uint64_t generation, Completion completion) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_ || fail_next_submit_ || pending_.count(id) != 0) {
      fail_next_submit_ = false;
      return false;
    }
    pending_.emplace(id, Pending{generation, std::move(completion), false});
    return true;
  }

  bool cancel(std::uint64_t id) override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = pending_.find(id);
    if (closed_ || it == pending_.end() || it->second.event_queued || fail_next_cancel_) {
      fail_next_cancel_ = false;
      return false;
    }
    it->second.event_queued = true;
    events_.push_back(Event{id, it->second.generation, UsbCompletionStatus::Cancelled});
    return true;
  }

  std::size_t poll_events() override {
    std::deque<Event> ready;
    std::vector<std::pair<Completion, Event>> callbacks;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ready.swap(events_);
      for (const auto & event : ready) {
        auto it = pending_.find(event.id);
        if (it == pending_.end()) continue;
        callbacks.emplace_back(it->second.completion, event);
        pending_.erase(it);
      }
    }
    for (auto & item : callbacks) {
      item.first(item.second.id, item.second.generation, item.second.status);
    }
    return callbacks.size();
  }

  bool complete(std::uint64_t id, UsbCompletionStatus status) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = pending_.find(id);
    if (closed_ || it == pending_.end() || it->second.event_queued) return false;
    it->second.event_queued = true;
    events_.push_back(Event{id, it->second.generation, status});
    return true;
  }

  void fail_next_submit() {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_next_submit_ = true;
  }

  void fail_next_cancel() {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_next_cancel_ = true;
  }

  bool close() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_.empty() || !events_.empty()) return false;
    closed_ = true;
    return true;
  }

  std::size_t pending_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_.size();
  }

private:
  struct Pending { std::uint64_t generation; Completion completion; bool event_queued; };
  struct Event { std::uint64_t id; std::uint64_t generation; UsbCompletionStatus status; };

  mutable std::mutex mutex_;
  std::map<std::uint64_t, Pending> pending_;
  std::deque<Event> events_;
  bool fail_next_submit_{false};
  bool fail_next_cancel_{false};
  bool closed_{false};
};

class TransferSession {
public:
  TransferSession(TransferLifecycle & lifecycle, IUsbTransferTransport & transport)
  : lifecycle_(lifecycle), transport_(transport) {}

  bool submit(std::uint64_t id) {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    const auto generation = lifecycle_.generation();
    if (!lifecycle_.register_transfer(id, generation)) return false;
    if (!lifecycle_.mark_submitted(id)) {
      lifecycle_.release(id);
      return false;
    }
    const bool ok = transport_.submit(id, generation,
      [this](std::uint64_t callback_id, std::uint64_t callback_generation, UsbCompletionStatus) {
        const auto result = lifecycle_.complete_callback(callback_id, callback_generation);
        if (result == CallbackDisposition::Drained || result == CallbackDisposition::StaleGeneration) {
          lifecycle_.release(callback_id);
        }
      });
    if (!ok) {
      lifecycle_.submission_failed(id);
      lifecycle_.release(id);
      return false;
    }
    return true;
  }

  bool cancel(std::uint64_t id) {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (!lifecycle_.request_cancel(id)) return false;
    return transport_.cancel(id);
  }

  std::size_t poll_events() {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    return transport_.poll_events();
  }

  bool shutdown(std::size_t max_polls = 1024) {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    const auto cancel_ids = lifecycle_.begin_shutdown();
    for (const auto id : cancel_ids) (void)transport_.cancel(id);
    for (std::size_t i = 0; i < max_polls && !lifecycle_.callbacks_drained(); ++i) {
      if (transport_.poll_events() == 0) break;
    }
    if (!lifecycle_.callbacks_drained()) return false;
    return transport_.close();
  }

private:
  std::mutex operation_mutex_;
  TransferLifecycle & lifecycle_;
  IUsbTransferTransport & transport_;
};

}  // namespace robodog_protocol
