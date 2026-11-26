#include "robodog_protocol/gnss_ingress_queue.hpp"

#include <cassert>
#include <cstdint>
#include <vector>

using namespace robodog_protocol;

static Frame frame(std::uint64_t sequence, std::uint64_t generation) {
  return Frame{Protocol::Ubx, {0xB5, 0x62}, sequence, generation,
               ChecksumStatus::NotApplicable};
}

int main() {
  {
    GnssIngressQueue queue(8, OverflowPolicy::DropOldest, "process-A");
    assert(queue.begin_generation(1, 100));
    assert(queue.push_frame(frame(0, 1), 110) == GnssIngressPushResult::Accepted);
    assert(queue.push_frame(frame(1, 1), 120) == GnssIngressPushResult::Accepted);
    assert(queue.mark_device_lost(1, 130));
    assert(queue.push_frame(frame(2, 1), 140) == GnssIngressPushResult::RejectedClosed);
    assert(queue.begin_generation(2, 150));
    assert(!queue.begin_generation(2, 155));
    assert(queue.push_frame(frame(1, 1), 156) ==
           GnssIngressPushResult::RejectedStaleGeneration);
    assert(queue.push_frame(frame(0, 2), 160) == GnssIngressPushResult::Accepted);

    std::vector<GnssIngressEvent> events;
    while (auto event = queue.try_pop()) events.push_back(std::move(*event));
    assert(events.size() == 6);
    assert(events[0].type == GnssIngressEventType::GenerationStarted);
    assert(events[1].type == GnssIngressEventType::Frame);
    assert(events[2].type == GnssIngressEventType::Frame);
    assert(events[3].type == GnssIngressEventType::DeviceLost);
    assert(events[4].type == GnssIngressEventType::GenerationStarted);
    assert(events[5].type == GnssIngressEventType::Frame);
    for (std::size_t i = 0; i < events.size(); ++i) {
      assert(events[i].ingress_sequence == i);
      assert(events[i].process_session_id == "process-A");
    }
    assert(events[0].connection_generation == 1);
    assert(events[3].connection_generation == 1);
    assert(events[4].connection_generation == 2);
    assert(events[5].frame.has_value());
    assert(events[5].frame->connection_generation == 2);
    const auto stats = queue.stats();
    assert(stats.accepted_control_events == 3);
    assert(stats.accepted_frame_events == 3);
    assert(stats.rejected_closed == 1);
    assert(stats.rejected_stale_generation == 1);
    assert(stats.current_size == 0);
    assert(stats.next_ingress_sequence == 6);
  }

  {
    GnssIngressQueue queue(3, OverflowPolicy::DropOldest, "process-B");
    assert(queue.begin_generation(7, 10));
    assert(queue.push_frame(frame(0, 7), 11) == GnssIngressPushResult::Accepted);
    assert(queue.push_frame(frame(1, 7), 12) == GnssIngressPushResult::Accepted);
    assert(queue.push_frame(frame(2, 7), 13) ==
           GnssIngressPushResult::AcceptedAfterDroppingOldestFrame);
    assert(queue.mark_device_lost(7, 14));
    auto first = queue.try_pop();
    auto second = queue.try_pop();
    auto third = queue.try_pop();
    assert(first && first->type == GnssIngressEventType::GenerationStarted);
    assert(first->dropped_frame_events_total == 0);
    assert(second && second->type == GnssIngressEventType::Frame);
    assert(second->dropped_frame_events_total == 2);
    assert(second->frame && second->frame->sequence == 2);
    assert(third && third->type == GnssIngressEventType::DeviceLost);
    assert(third->dropped_frame_events_total == 2);
    assert(queue.stats().dropped_oldest_frame_events == 2);
  }

  {
    GnssIngressQueue queue(1, OverflowPolicy::DropOldest, "process-C");
    assert(queue.begin_generation(1, 1));
    // A full control-only queue cannot evict a control event. The queue must
    // fail closed rather than silently losing DeviceLost.
    assert(!queue.mark_device_lost(1, 2));
    const auto stats = queue.stats();
    assert(stats.control_event_overflow_failures == 1);
    assert(stats.closed);
    assert(!stats.accepting_frames);
  }

  {
    GnssIngressQueue queue(1, OverflowPolicy::DropOldest, "process-E");
    assert(queue.begin_generation(1, 10));
    // The only slot is occupied by GenerationStarted. stop must report that
    // Stopped could not be recorded, and a later retry must succeed after drain.
    assert(!queue.close(20));
    assert(queue.stats().closed);
    assert(!queue.stats().stopped_event_recorded);
    auto started = queue.try_pop();
    assert(started && started->type == GnssIngressEventType::GenerationStarted);
    assert(queue.close(30));
    assert(queue.stats().stopped_event_recorded);
    auto stopped = queue.try_pop();
    assert(stopped && stopped->type == GnssIngressEventType::Stopped);
    assert(stopped->ingress_sequence == 1);
    assert(queue.close(40));
    assert(queue.stats().accepted_control_events == 2);
  }

  {
    GnssIngressQueue queue(1, OverflowPolicy::DropOldest, "process-F");
    assert(queue.begin_generation(1, 10));
    // A terminal event that cannot fit is remembered and can be retried.
    assert(!queue.mark_device_lost(1, 20));
    assert(!queue.stats().terminal_event_recorded);
    auto started = queue.try_pop();
    assert(started && started->type == GnssIngressEventType::GenerationStarted);
    assert(queue.mark_device_lost(1, 30));
    assert(queue.stats().terminal_event_recorded);
    assert(queue.mark_device_lost(1, 31));  // duplicate callback is idempotent
    assert(!queue.mark_transport_fault(1, 32));  // first terminal cause wins
    auto lost = queue.try_pop();
    assert(lost && lost->type == GnssIngressEventType::DeviceLost);
    assert(lost->ingress_sequence == 1);
    assert(queue.close(40));  // room is now available for Stopped
    auto stopped = queue.try_pop();
    assert(stopped && stopped->type == GnssIngressEventType::Stopped);
    assert(stopped->ingress_sequence == 2);
    assert(queue.stats().accepted_control_events == 3);
  }

  {
    GnssIngressQueue queue(4, OverflowPolicy::DropOldest, "process-G");
    assert(queue.begin_generation(1, 10));
    assert(!queue.begin_generation(2, 11));  // active generation cannot be replaced
    assert(queue.mark_transport_fault(1, 12));
    assert(queue.begin_generation(2, 13));   // terminal event fences old generation
  }

  {
    GnssIngressQueue queue(1, OverflowPolicy::DropOldest, "process-H");
    assert(queue.begin_generation(1, 10));
    assert(!queue.mark_device_lost(1, 11));
    auto started = queue.try_pop();
    assert(started && started->type == GnssIngressEventType::GenerationStarted);
    assert(queue.mark_device_lost(1, 12));
    assert(!queue.begin_generation(2, 13));  // no room for GenerationStarted
    assert(queue.stats().current_generation == 1);  // rejected start is atomic
    auto lost = queue.try_pop();
    assert(lost && lost->type == GnssIngressEventType::DeviceLost);
    assert(queue.begin_generation(2, 14));
    auto next = queue.try_pop();
    assert(next && next->type == GnssIngressEventType::GenerationStarted);
    assert(next->connection_generation == 2);
  }

  {
    GnssIngressQueue queue(4, OverflowPolicy::DropOldest, "process-I");
    assert(queue.begin_generation(1, 10));
    assert(queue.close(11));
    assert(!queue.mark_device_lost(1, 12));
    assert(!queue.mark_transport_fault(1, 13));
    auto started = queue.try_pop();
    auto stopped = queue.try_pop();
    assert(started && started->type == GnssIngressEventType::GenerationStarted);
    assert(stopped && stopped->type == GnssIngressEventType::Stopped);
    assert(!queue.try_pop());
    assert(queue.stats().accepted_control_events == 2);
  }

  {
    GnssIngressQueue queue(2, OverflowPolicy::RejectNewest, "process-D");
    assert(queue.begin_generation(1, 1));
    assert(queue.push_frame(frame(0, 1), 2) == GnssIngressPushResult::Accepted);
    assert(queue.push_frame(frame(1, 1), 3) == GnssIngressPushResult::RejectedFull);
    assert(queue.stats().accepted_frame_events == 1);
    assert(queue.stats().rejected_full_frame_events == 1);
  }

  return 0;
}
