#include "robodog_protocol/bounded_frame_queue.hpp"

#include <atomic>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace robodog_protocol;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

Frame make_frame(std::uint64_t sequence, std::uint64_t generation = 1) {
  return Frame{Protocol::Ubx, {0xB5, 0x62}, sequence, generation};
}

void test_reject_newest_policy() {
  BoundedFrameQueue queue(2, OverflowPolicy::RejectNewest, 1);
  require(queue.push(make_frame(1)) == QueuePushResult::Accepted, "first push rejected");
  require(queue.push(make_frame(2)) == QueuePushResult::Accepted, "second push rejected");
  require(queue.push(make_frame(3)) == QueuePushResult::RejectedFull, "full queue did not reject newest");
  require(queue.try_pop()->sequence == 1, "reject-newest changed FIFO order");
  const auto stats = queue.stats();
  require(stats.rejected_full == 1 && stats.current_size == 1 && stats.high_watermark == 2,
          "reject-newest stats mismatch");
}

void test_drop_oldest_policy() {
  BoundedFrameQueue queue(2, OverflowPolicy::DropOldest, 1);
  queue.push(make_frame(1));
  queue.push(make_frame(2));
  require(queue.push(make_frame(3)) == QueuePushResult::AcceptedAfterDroppingOldest,
          "drop-oldest result mismatch");
  require(queue.try_pop()->sequence == 2 && queue.try_pop()->sequence == 3,
          "drop-oldest FIFO contents mismatch");
  require(queue.stats().dropped_oldest == 1, "drop-oldest counter mismatch");
}

void test_generation_fence_and_close() {
  BoundedFrameQueue queue(4, OverflowPolicy::DropOldest, 1);
  queue.push(make_frame(1));
  queue.reset_generation(2);
  require(!queue.try_pop().has_value(), "generation reset retained stale frame");
  require(queue.push(make_frame(2, 1)) == QueuePushResult::RejectedStaleGeneration,
          "old generation frame accepted");
  require(queue.push(make_frame(3, 2)) == QueuePushResult::Accepted,
          "new generation frame rejected");
  queue.close();
  require(queue.push(make_frame(4, 2)) == QueuePushResult::RejectedClosed,
          "closed queue accepted a push");
  require(queue.try_pop()->sequence == 3, "close should leave queued frame drainable");
  require(!queue.try_pop().has_value(), "closed queue should drain to empty");
  const auto stats = queue.stats();
  require(stats.rejected_stale_generation == 1 && stats.rejected_closed == 1 &&
          stats.connection_generation == 2 && stats.closed,
          "generation/close stats mismatch");
}

void test_concurrent_producers_and_consumer() {
  constexpr std::uint64_t kGeneration = 7;
  constexpr std::size_t kProducerCount = 4;
  constexpr std::size_t kFramesPerProducer = 1000;
  BoundedFrameQueue queue(kProducerCount * kFramesPerProducer,
                          OverflowPolicy::RejectNewest, kGeneration);
  std::atomic<std::size_t> producers_done{0};
  std::atomic<std::size_t> consumed{0};
  std::vector<std::thread> producers;
  for (std::size_t p = 0; p < kProducerCount; ++p) {
    producers.emplace_back([&, p] {
      for (std::size_t i = 0; i < kFramesPerProducer; ++i) {
        const auto sequence = static_cast<std::uint64_t>(p * kFramesPerProducer + i);
        const auto result = queue.push(make_frame(sequence, kGeneration));
        if (result != QueuePushResult::Accepted) std::terminate();
      }
      producers_done.fetch_add(1, std::memory_order_release);
    });
  }
  std::thread consumer([&] {
    while (producers_done.load(std::memory_order_acquire) != kProducerCount ||
           queue.stats().current_size != 0) {
      auto frame = queue.try_pop();
      if (frame) consumed.fetch_add(1, std::memory_order_relaxed);
      else std::this_thread::yield();
    }
  });
  for (auto& producer : producers) producer.join();
  consumer.join();
  const auto stats = queue.stats();
  require(consumed.load() == kProducerCount * kFramesPerProducer,
          "concurrent queue lost frames");
  require(stats.accepted == consumed.load() && stats.popped == consumed.load() &&
          stats.rejected_full == 0 && stats.current_size == 0,
          "concurrent queue counters mismatch");
}

void test_concurrent_overflow_is_bounded() {
  BoundedFrameQueue queue(16, OverflowPolicy::DropOldest, 9);
  std::vector<std::thread> producers;
  for (std::uint64_t p = 0; p < 4; ++p) {
    producers.emplace_back([&, p] {
      for (std::uint64_t i = 0; i < 1000; ++i)
        (void)queue.push(make_frame(p * 1000 + i, 9));
    });
  }
  for (auto& producer : producers) producer.join();
  const auto stats = queue.stats();
  require(stats.current_size <= queue.capacity() && stats.current_size == 16,
          "overflow queue exceeded capacity");
  require(stats.accepted == 4000 && stats.dropped_oldest == 3984,
          "overflow queue accounting mismatch");
}

int main() {
  try {
    test_reject_newest_policy();
    test_drop_oldest_policy();
    test_generation_fence_and_close();
    test_concurrent_producers_and_consumer();
    test_concurrent_overflow_is_bounded();
    std::cout << "5 bounded queue test groups passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
