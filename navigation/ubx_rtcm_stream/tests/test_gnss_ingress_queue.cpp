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
    assert(second && second->type == GnssIngressEventType::Frame);
    assert(second->frame && second->frame->sequence == 2);
    assert(third && third->type == GnssIngressEventType::DeviceLost);
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
    GnssIngressQueue queue(2, OverflowPolicy::RejectNewest, "process-D");
    assert(queue.begin_generation(1, 1));
    assert(queue.push_frame(frame(0, 1), 2) == GnssIngressPushResult::Accepted);
    assert(queue.push_frame(frame(1, 1), 3) == GnssIngressPushResult::RejectedFull);
    assert(queue.stats().accepted_frame_events == 1);
    assert(queue.stats().rejected_full_frame_events == 1);
  }

  return 0;
}
