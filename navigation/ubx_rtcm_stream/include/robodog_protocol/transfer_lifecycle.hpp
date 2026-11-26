#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <vector>

namespace robodog_protocol {

enum class TransferState {
  Allocated,
  Submitted,
  CancelRequested,
  CallbackDrained,
  SubmitFailed,
  Released
};

enum class CallbackDisposition {
  Drained,
  StaleGeneration,
  UnknownTransfer,
  DuplicateCallback,
  InvalidState
};

struct TransferSnapshot {
  std::uint64_t id{};
  std::uint64_t generation{};
  TransferState state{TransferState::Allocated};
};

class TransferLifecycle {
public:
  explicit TransferLifecycle(std::uint64_t initial_generation = 1)
  : generation_(initial_generation) {}

  bool register_transfer(std::uint64_t id, std::uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!accepting_ || generation != generation_ || records_.count(id) != 0) return false;
    records_.emplace(id, TransferSnapshot{id, generation, TransferState::Allocated});
    return true;
  }

  bool mark_submitted(std::uint64_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = records_.find(id);
    if (it == records_.end() || it->second.state != TransferState::Allocated || !accepting_) return false;
    it->second.state = TransferState::Submitted;
    return true;
  }

  bool request_cancel(std::uint64_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = records_.find(id);
    if (it == records_.end() || it->second.state != TransferState::Submitted) return false;
    it->second.state = TransferState::CancelRequested;
    return true;
  }

  CallbackDisposition complete_callback(std::uint64_t id, std::uint64_t callback_generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = records_.find(id);
    if (it == records_.end()) return CallbackDisposition::UnknownTransfer;
    if (it->second.state == TransferState::CallbackDrained ||
        it->second.state == TransferState::Released) return CallbackDisposition::DuplicateCallback;
    if (it->second.state != TransferState::Submitted &&
        it->second.state != TransferState::CancelRequested) return CallbackDisposition::InvalidState;
    const bool stale = callback_generation != it->second.generation ||
                       callback_generation != generation_;
    it->second.state = TransferState::CallbackDrained;
    cv_.notify_all();
    return stale ? CallbackDisposition::StaleGeneration : CallbackDisposition::Drained;
  }

  bool submission_failed(std::uint64_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = records_.find(id);
    if (it == records_.end() || it->second.state != TransferState::Submitted) return false;
    it->second.state = TransferState::SubmitFailed;
    cv_.notify_all();
    return true;
  }

  bool release(std::uint64_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = records_.find(id);
    if (it == records_.end()) return false;
    if (it->second.state != TransferState::CallbackDrained &&
        it->second.state != TransferState::SubmitFailed &&
        it->second.state != TransferState::Allocated) return false;
    it->second.state = TransferState::Released;
    cv_.notify_all();
    return true;
  }

  // Stops new submissions and returns transfers for which cancellation should be requested.
  std::vector<std::uint64_t> begin_shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    accepting_ = false;
    std::vector<std::uint64_t> cancel;
    for (auto & entry : records_) {
      if (entry.second.state == TransferState::Submitted) {
        entry.second.state = TransferState::CancelRequested;
        cancel.push_back(entry.first);
      }
    }
    return cancel;
  }

  bool callbacks_drained() const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & entry : records_) {
      if (entry.second.state == TransferState::Submitted ||
          entry.second.state == TransferState::CancelRequested) return false;
    }
    return true;
  }

  bool wait_for_callbacks(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [this] {
      for (const auto & entry : records_) {
        if (entry.second.state == TransferState::Submitted ||
            entry.second.state == TransferState::CancelRequested) return false;
      }
      return true;
    });
  }

  bool begin_generation(std::uint64_t next_generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (accepting_ || next_generation <= generation_) return false;
    for (const auto & entry : records_) {
      if (entry.second.state != TransferState::Released) return false;
    }
    records_.clear();
    generation_ = next_generation;
    accepting_ = true;
    return true;
  }

  std::optional<TransferSnapshot> snapshot(std::uint64_t id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = records_.find(id);
    if (it == records_.end()) return std::nullopt;
    return it->second;
  }

  std::uint64_t generation() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return generation_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::map<std::uint64_t, TransferSnapshot> records_;
  std::uint64_t generation_;
  bool accepting_{true};
};

}  // namespace robodog_protocol
