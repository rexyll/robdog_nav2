#include "robodog_protocol/fake_usb_transport.hpp"

#include <cassert>
#include <cstdint>
#include <atomic>
#include <thread>

using robodog_protocol::FakeUsbTransport;
using robodog_protocol::TransferLifecycle;
using robodog_protocol::TransferSession;
using robodog_protocol::TransferState;
using robodog_protocol::UsbCompletionStatus;

int main() {
  {
    TransferLifecycle lifecycle(1);
    FakeUsbTransport transport;
    TransferSession session(lifecycle, transport);
    assert(session.submit(1));
    assert(transport.complete(1, UsbCompletionStatus::Completed));
    assert(session.poll_events() == 1);
    assert(lifecycle.snapshot(1)->state == TransferState::Released);
    assert(session.shutdown());
  }
  {
    TransferLifecycle lifecycle(4);
    FakeUsbTransport transport;
    TransferSession session(lifecycle, transport);
    assert(session.submit(10));
    assert(session.cancel(10));
    assert(transport.poll_events() == 1);
    assert(lifecycle.snapshot(10)->state == TransferState::Released);
    assert(session.shutdown());
  }
  {
    const UsbCompletionStatus statuses[] = {
      UsbCompletionStatus::NoDevice, UsbCompletionStatus::Error, UsbCompletionStatus::Cancelled
    };
    std::uint64_t id = 20;
    for (const auto status : statuses) {
      TransferLifecycle lifecycle(1);
      FakeUsbTransport transport;
      TransferSession session(lifecycle, transport);
      assert(session.submit(id));
      assert(transport.complete(id, status));
      assert(transport.poll_events() == 1);
      assert(lifecycle.snapshot(id)->state == TransferState::Released);
      assert(session.shutdown());
      ++id;
    }
  }
  {
    TransferLifecycle lifecycle(1);
    FakeUsbTransport transport;
    TransferSession session(lifecycle, transport);
    transport.fail_next_submit();
    assert(!session.submit(30));
    assert(lifecycle.snapshot(30)->state == TransferState::Released);
    assert(transport.pending_count() == 0);
    assert(session.shutdown());
  }
  {
    TransferLifecycle lifecycle(1);
    FakeUsbTransport transport;
    TransferSession session(lifecycle, transport);
    assert(session.submit(40));
    transport.fail_next_cancel();
    assert(!session.cancel(40));
    assert(lifecycle.snapshot(40)->state == TransferState::CancelRequested);
    assert(transport.complete(40, UsbCompletionStatus::Completed));
    assert(transport.poll_events() == 1);
    assert(lifecycle.snapshot(40)->state == TransferState::Released);
    assert(session.shutdown());
  }
  {
    TransferLifecycle lifecycle(9);
    FakeUsbTransport transport;
    TransferSession session(lifecycle, transport);
    assert(session.submit(50));
    assert(session.shutdown());
    assert(lifecycle.snapshot(50)->state == TransferState::Released);
    assert(transport.pending_count() == 0);
  }
  {
    // The fake captures generation at submit/event enqueue, not at callback time.
    FakeUsbTransport transport;
    std::uint64_t observed_generation = 0;
    assert(transport.submit(90, 42, [&](std::uint64_t, std::uint64_t generation, UsbCompletionStatus) {
      observed_generation = generation;
    }));
    assert(transport.complete(90, UsbCompletionStatus::NoDevice));
    assert(transport.poll_events() == 1);
    assert(observed_generation == 42);
    assert(transport.close());
  }
  {
    // A completion already queued wins over a concurrent shutdown cancellation;
    // the fake must not enqueue two terminal callbacks for one transfer.
    TransferLifecycle lifecycle(1);
    FakeUsbTransport transport;
    TransferSession session(lifecycle, transport);
    assert(session.submit(91));
    assert(transport.complete(91, UsbCompletionStatus::Completed));
    assert(session.shutdown());
    assert(lifecycle.snapshot(91)->state == TransferState::Released);
  }
  {
    // Serialize submit and shutdown so no transfer can be submitted after shutdown's
    // cancellation snapshot has been taken.
    for (std::uint64_t id = 100; id < 200; ++id) {
      TransferLifecycle lifecycle(1);
      FakeUsbTransport transport;
      TransferSession session(lifecycle, transport);
      std::atomic<bool> start{false};
      bool submitted = false;
      bool shutdown_ok = false;
      std::thread submit_thread([&] {
        while (!start.load(std::memory_order_acquire)) {}
        submitted = session.submit(id);
      });
      std::thread shutdown_thread([&] {
        while (!start.load(std::memory_order_acquire)) {}
        shutdown_ok = session.shutdown();
      });
      start.store(true, std::memory_order_release);
      submit_thread.join();
      shutdown_thread.join();
      assert(shutdown_ok);
      assert(transport.pending_count() == 0);
      if (submitted) assert(lifecycle.snapshot(id)->state == TransferState::Released);
    }
  }
  return 0;
}
