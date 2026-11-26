#pragma once

#include "robodog_protocol/stream_decoder.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

namespace robodog_protocol {

enum class OverflowPolicy : std::uint8_t { DropOldest, RejectNewest };
enum class QueuePushResult : std::uint8_t {
  Accepted,
  AcceptedAfterDroppingOldest,
  RejectedFull,
  RejectedStaleGeneration,
  RejectedClosed
};

struct QueueStats {
  std::uint64_t accepted{0};
  std::uint64_t popped{0};
  std::uint64_t dropped_oldest{0};
  std::uint64_t rejected_full{0};
  std::uint64_t rejected_stale_generation{0};
  std::uint64_t rejected_closed{0};
  std::size_t current_size{0};
  std::size_t high_watermark{0};
  bool closed{false};
  std::uint64_t connection_generation{0};
};

// A non-blocking, bounded handoff queue. Producers and consumers may run on
// different executor threads; all queue state and statistics share one mutex.
class BoundedFrameQueue {
 public:
  BoundedFrameQueue(std::size_t capacity, OverflowPolicy policy,
                    std::uint64_t connection_generation)
      : capacity_(capacity), policy_(policy),
        connection_generation_(connection_generation) {
    if (capacity_ == 0) throw std::invalid_argument("queue capacity must be > 0");
  }

  QueuePushResult push(Frame frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      ++stats_.rejected_closed;
      return QueuePushResult::RejectedClosed;
    }
    if (frame.connection_generation != connection_generation_) {
      ++stats_.rejected_stale_generation;
      return QueuePushResult::RejectedStaleGeneration;
    }
    bool dropped = false;
    if (queue_.size() == capacity_) {
      if (policy_ == OverflowPolicy::RejectNewest) {
        ++stats_.rejected_full;
        return QueuePushResult::RejectedFull;
      }
      queue_.pop_front();
      ++stats_.dropped_oldest;
      dropped = true;
    }
    queue_.push_back(std::move(frame));
    ++stats_.accepted;
    update_watermark_locked();
    return dropped ? QueuePushResult::AcceptedAfterDroppingOldest
                   : QueuePushResult::Accepted;
  }

  std::optional<Frame> try_pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) return std::nullopt;
    Frame frame = std::move(queue_.front());
    queue_.pop_front();
    ++stats_.popped;
    update_watermark_locked();
    return frame;
  }

  // A generation transition is an atomic fence: pending old-generation frames
  // are discarded and subsequent pushes must carry the new generation.
  void reset_generation(std::uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
    connection_generation_ = generation;
    closed_ = false;
    update_watermark_locked();
  }

  // close() rejects new pushes but leaves already queued frames drainable.
  void close() {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
    update_watermark_locked();
  }

  QueueStats stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    QueueStats snapshot = stats_;
    snapshot.current_size = queue_.size();
    snapshot.closed = closed_;
    snapshot.connection_generation = connection_generation_;
    return snapshot;
  }

  std::size_t capacity() const noexcept { return capacity_; }

 private:
  void update_watermark_locked() {
    stats_.high_watermark = std::max(stats_.high_watermark, queue_.size());
  }

  const std::size_t capacity_;
  const OverflowPolicy policy_;
  mutable std::mutex mutex_;
  std::deque<Frame> queue_;
  QueueStats stats_;
  std::uint64_t connection_generation_;
  bool closed_{false};
};

}  // namespace robodog_protocol
