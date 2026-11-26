#pragma once

#include "robodog_protocol/bounded_frame_queue.hpp"

#include <algorithm>
#include <cstddef>
#include <cctype>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace robodog_protocol {

enum class GnssIngressEventType : std::uint8_t {
  GenerationStarted,
  Frame,
  DeviceLost,
  TransportFault,
  Stopped
};

struct GnssIngressEvent {
  GnssIngressEventType type{GnssIngressEventType::Frame};
  std::string process_session_id;
  std::uint64_t connection_generation{0};
  std::uint64_t ingress_sequence{0};
  std::uint64_t receive_monotonic_ns{0};
  // Prefix count of DropOldest frame evictions whose dropped frame sequence
  // precedes this event's ingress_sequence. Later drops never rewrite earlier events.
  std::uint64_t dropped_frame_events_total{0};
  std::optional<Frame> frame;
};

struct GnssIngressQueueStats {
  std::uint64_t accepted_frame_events{0};
  std::uint64_t accepted_control_events{0};
  std::uint64_t popped_frame_events{0};
  std::uint64_t popped_control_events{0};
  std::uint64_t dropped_oldest_frame_events{0};
  std::uint64_t rejected_full_frame_events{0};
  std::uint64_t rejected_stale_generation{0};
  std::uint64_t rejected_closed{0};
  std::uint64_t control_event_overflow_failures{0};
  std::size_t current_size{0};
  std::size_t current_frame_events{0};
  std::size_t high_watermark{0};
  std::size_t frame_high_watermark{0};
  std::uint64_t current_generation{0};
  std::uint64_t next_ingress_sequence{0};
  bool has_generation{false};
  bool accepting_frames{false};
  bool device_lost{false};
  bool closed{false};
  bool stopped_event_recorded{false};
  bool terminal_event_recorded{false};
};

enum class GnssIngressPushResult : std::uint8_t {
  Accepted,
  AcceptedAfterDroppingOldestFrame,
  RejectedFull,
  RejectedStaleGeneration,
  RejectedClosed
};

// One bounded, ordered event stream for receiver frames and lifecycle events.
// Control events have priority over data: when full they may evict the oldest
// frame, but are never silently dropped. If the queue contains only control
// events and cannot accept another control event, the queue fails closed and
// reports control_event_overflow_failures.
class GnssIngressQueue {
 public:
  GnssIngressQueue(std::size_t capacity, OverflowPolicy frame_overflow_policy,
                   std::string process_session_id)
      : capacity_(capacity), frame_overflow_policy_(frame_overflow_policy),
        process_session_id_(std::move(process_session_id)) {
    if (capacity_ == 0) throw std::invalid_argument("ingress queue capacity must be > 0");
    if (process_session_id_.empty() ||
        std::all_of(process_session_id_.begin(), process_session_id_.end(),
                    [](unsigned char c) { return std::isspace(c) != 0; })) {
      throw std::invalid_argument("process_session_id must be non-empty");
    }
  }

  bool begin_generation(std::uint64_t generation,
                        std::uint64_t receive_monotonic_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_generation_ && generation <= current_generation_) return false;
    if (has_generation_ && !stopped_event_recorded_ &&
        !(terminal_event_type_.has_value() && terminal_event_recorded_)) {
      return false;
    }
    // Reserve room (or an evictable frame) before mutating the active
    // generation, so a rejected GenerationStarted event cannot advance state.
    if (events_.size() >= capacity_ && frame_event_count_locked() == 0) {
      ++stats_.control_event_overflow_failures;
      accepting_frames_ = false;
      closed_ = true;
      return false;
    }
    current_generation_ = generation;
    has_generation_ = true;
    accepting_frames_ = true;
    device_lost_ = false;
    closed_ = false;
    stopped_event_recorded_ = false;
    terminal_event_type_.reset();
    terminal_event_recorded_ = false;
    return push_control_locked(
        GnssIngressEventType::GenerationStarted, generation,
        receive_monotonic_ns);
  }

  GnssIngressPushResult push_frame(Frame frame,
                                   std::uint64_t receive_monotonic_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_ || !accepting_frames_) {
      ++stats_.rejected_closed;
      return GnssIngressPushResult::RejectedClosed;
    }
    if (!has_generation_ ||
        frame.connection_generation != current_generation_) {
      ++stats_.rejected_stale_generation;
      return GnssIngressPushResult::RejectedStaleGeneration;
    }

    bool dropped = false;
    if (events_.size() >= capacity_) {
      if (frame_overflow_policy_ == OverflowPolicy::RejectNewest ||
          !drop_oldest_frame_locked()) {
        ++stats_.rejected_full_frame_events;
        return GnssIngressPushResult::RejectedFull;
      }
      dropped = true;
    }
    GnssIngressEvent event;
    event.type = GnssIngressEventType::Frame;
    event.process_session_id = process_session_id_;
    event.connection_generation = current_generation_;
    event.ingress_sequence = next_sequence_++;
    event.receive_monotonic_ns = receive_monotonic_ns;
    event.dropped_frame_events_total = stats_.dropped_oldest_frame_events;
    event.frame = std::move(frame);
    events_.push_back(std::move(event));
    ++stats_.accepted_frame_events;
    stats_.frame_high_watermark = std::max(
        stats_.frame_high_watermark, frame_event_count_locked());
    update_watermark_locked();
    return dropped ? GnssIngressPushResult::AcceptedAfterDroppingOldestFrame
                   : GnssIngressPushResult::Accepted;
  }

  bool mark_device_lost(std::uint64_t generation,
                        std::uint64_t receive_monotonic_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_generation_ || generation != current_generation_) {
      ++stats_.rejected_stale_generation;
      return false;
    }
    if (closed_ && !terminal_event_type_) return false;
    if (terminal_event_type_ &&
        *terminal_event_type_ != GnssIngressEventType::DeviceLost) {
      return false;
    }
    accepting_frames_ = false;
    device_lost_ = true;
    terminal_event_type_ = GnssIngressEventType::DeviceLost;
    if (terminal_event_recorded_) return true;
    terminal_event_recorded_ = push_control_locked(
        GnssIngressEventType::DeviceLost, generation, receive_monotonic_ns);
    return terminal_event_recorded_;
  }

  bool mark_transport_fault(std::uint64_t generation,
                            std::uint64_t receive_monotonic_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_generation_ || generation != current_generation_) {
      ++stats_.rejected_stale_generation;
      return false;
    }
    if (closed_ && !terminal_event_type_) return false;
    if (terminal_event_type_ &&
        *terminal_event_type_ != GnssIngressEventType::TransportFault) {
      return false;
    }
    accepting_frames_ = false;
    terminal_event_type_ = GnssIngressEventType::TransportFault;
    if (terminal_event_recorded_) return true;
    terminal_event_recorded_ = push_control_locked(
        GnssIngressEventType::TransportFault, generation,
        receive_monotonic_ns);
    return terminal_event_recorded_;
  }

  bool close(std::uint64_t receive_monotonic_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_event_recorded_) return true;
    accepting_frames_ = false;
    if (terminal_event_type_ && !terminal_event_recorded_) {
      terminal_event_recorded_ = push_control_locked(
          *terminal_event_type_, current_generation_, receive_monotonic_ns);
      if (!terminal_event_recorded_) {
        closed_ = true;
        return false;
      }
    }
    const bool recorded = has_generation_
        ? push_control_locked(GnssIngressEventType::Stopped,
                              current_generation_, receive_monotonic_ns)
        : true;
    closed_ = true;
    stopped_event_recorded_ = recorded;
    return recorded;
  }

  std::optional<GnssIngressEvent> try_pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (events_.empty()) return std::nullopt;
    GnssIngressEvent event = std::move(events_.front());
    events_.pop_front();
    if (event.type == GnssIngressEventType::Frame) {
      ++stats_.popped_frame_events;
    } else {
      ++stats_.popped_control_events;
    }
    return event;
  }

  GnssIngressQueueStats stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto snapshot = stats_;
    snapshot.current_size = events_.size();
    snapshot.current_frame_events = frame_event_count_locked();
    snapshot.current_generation = current_generation_;
    snapshot.next_ingress_sequence = next_sequence_;
    snapshot.has_generation = has_generation_;
    snapshot.accepting_frames = accepting_frames_;
    snapshot.device_lost = device_lost_;
    snapshot.closed = closed_;
    snapshot.stopped_event_recorded = stopped_event_recorded_;
    snapshot.terminal_event_recorded = terminal_event_recorded_;
    return snapshot;
  }

  QueueStats frame_queue_stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    QueueStats result;
    result.accepted = stats_.accepted_frame_events;
    result.popped = stats_.popped_frame_events;
    result.dropped_oldest = stats_.dropped_oldest_frame_events;
    result.rejected_full = stats_.rejected_full_frame_events;
    result.rejected_stale_generation = stats_.rejected_stale_generation;
    result.rejected_closed = stats_.rejected_closed;
    result.current_size = frame_event_count_locked();
    result.high_watermark = stats_.frame_high_watermark;
    result.closed = closed_;
    result.connection_generation = current_generation_;
    return result;
  }

 private:
  bool drop_oldest_frame_locked() {
    for (auto it = events_.begin(); it != events_.end(); ++it) {
      if (it->type == GnssIngressEventType::Frame) {
        const auto dropped_sequence = it->ingress_sequence;
        events_.erase(it);
        ++stats_.dropped_oldest_frame_events;
        // Stamp only later events. Earlier queued lifecycle events must not
        // inherit drops whose sequence lies after them.
        for (auto & event : events_) {
          if (event.ingress_sequence > dropped_sequence) {
            event.dropped_frame_events_total = stats_.dropped_oldest_frame_events;
          }
        }
        return true;
      }
    }
    return false;
  }

  bool push_control_locked(GnssIngressEventType type,
                           std::uint64_t generation,
                           std::uint64_t receive_monotonic_ns) {
    if (events_.size() >= capacity_ && !drop_oldest_frame_locked()) {
      ++stats_.control_event_overflow_failures;
      accepting_frames_ = false;
      closed_ = true;
      return false;
    }
    GnssIngressEvent event;
    event.type = type;
    event.process_session_id = process_session_id_;
    event.connection_generation = generation;
    event.ingress_sequence = next_sequence_++;
    event.receive_monotonic_ns = receive_monotonic_ns;
    event.dropped_frame_events_total = stats_.dropped_oldest_frame_events;
    events_.push_back(std::move(event));
    ++stats_.accepted_control_events;
    update_watermark_locked();
    return true;
  }

  std::size_t frame_event_count_locked() const {
    return static_cast<std::size_t>(std::count_if(
        events_.begin(), events_.end(),
        [](const GnssIngressEvent & event) {
          return event.type == GnssIngressEventType::Frame;
        }));
  }

  void update_watermark_locked() {
    stats_.high_watermark = std::max(stats_.high_watermark, events_.size());
  }

  const std::size_t capacity_;
  const OverflowPolicy frame_overflow_policy_;
  const std::string process_session_id_;
  mutable std::mutex mutex_;
  std::deque<GnssIngressEvent> events_;
  GnssIngressQueueStats stats_;
  std::uint64_t current_generation_{0};
  std::uint64_t next_sequence_{0};
  bool has_generation_{false};
  bool accepting_frames_{false};
  bool device_lost_{false};
  bool closed_{false};
  bool stopped_event_recorded_{false};
  std::optional<GnssIngressEventType> terminal_event_type_;
  bool terminal_event_recorded_{false};
};

}  // namespace robodog_protocol
