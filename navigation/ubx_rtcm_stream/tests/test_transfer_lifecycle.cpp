#include "robodog_protocol/transfer_lifecycle.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using robodog_protocol::CallbackDisposition;
using robodog_protocol::TransferLifecycle;
using robodog_protocol::TransferState;

int main() {
  {
    TransferLifecycle lifecycle(7);
    assert(lifecycle.register_transfer(1, 7));
    assert(lifecycle.mark_submitted(1));
    assert(!lifecycle.release(1));  // Never free before callback completion.
    assert(lifecycle.request_cancel(1));
    assert(!lifecycle.request_cancel(1));  // Idempotent cancellation request.
    assert(lifecycle.complete_callback(1, 7) == CallbackDisposition::Drained);
    assert(lifecycle.callbacks_drained());
    assert(lifecycle.release(1));
    assert(!lifecycle.release(1));
  }

  {
    TransferLifecycle lifecycle(2);
    assert(lifecycle.register_transfer(10, 2));
    assert(lifecycle.mark_submitted(10));
    const auto cancel = lifecycle.begin_shutdown();
    assert(cancel.size() == 1 && cancel.front() == 10);
    assert(!lifecycle.register_transfer(11, 2));
    assert(!lifecycle.callbacks_drained());
    assert(lifecycle.complete_callback(10, 2) == CallbackDisposition::Drained);
    assert(lifecycle.wait_for_callbacks(10ms));
    assert(lifecycle.release(10));
    assert(lifecycle.begin_generation(3));
    assert(lifecycle.generation() == 3);
    assert(lifecycle.register_transfer(12, 3));
  }

  {
    TransferLifecycle lifecycle(4);
    assert(lifecycle.register_transfer(20, 4));
    assert(lifecycle.mark_submitted(20));
    assert(lifecycle.complete_callback(20, 99) == CallbackDisposition::StaleGeneration);
    assert(lifecycle.complete_callback(20, 4) == CallbackDisposition::DuplicateCallback);
    assert(lifecycle.release(20));
  }

  // Race cancel against completion. Exactly one terminal callback transition is accepted.
  for (std::uint64_t id = 100; id < 300; ++id) {
    TransferLifecycle lifecycle(1);
    assert(lifecycle.register_transfer(id, 1));
    assert(lifecycle.mark_submitted(id));
    std::atomic<bool> go{false};
    bool cancel_result = false;
    CallbackDisposition callback_result = CallbackDisposition::InvalidState;
    std::thread cancel_thread([&] {
      while (!go.load(std::memory_order_acquire)) {}
      cancel_result = lifecycle.request_cancel(id);
    });
    std::thread callback_thread([&] {
      while (!go.load(std::memory_order_acquire)) {}
      callback_result = lifecycle.complete_callback(id, 1);
    });
    go.store(true, std::memory_order_release);
    cancel_thread.join();
    callback_thread.join();
    assert(callback_result == CallbackDisposition::Drained);
    assert(lifecycle.callbacks_drained());
    assert(lifecycle.release(id));
    (void)cancel_result;  // Either ordering is valid; callback drain is the release gate.
  }

  return 0;
}
