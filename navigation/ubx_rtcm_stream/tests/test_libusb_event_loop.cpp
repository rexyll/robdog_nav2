#include "robodog_protocol/libusb_event_loop.hpp"

#include <atomic>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

using robodog_protocol::LibusbEventLoop;
using robodog_protocol::LibusbTransferRequest;
using robodog_protocol::LibusbTransferTransport;
using robodog_protocol::LibusbCompletionStatus;
using robodog_protocol::UsbTransferDirection;

struct NativeTransfer {
  LibusbTransferTransport::NativeCallback callback{};
  void * user_data{};
  std::uint8_t * buffer{};
  int length{};
  int status{};
  int actual_length{};
};

struct FakeUsb {
  std::mutex mutex;
  std::vector<NativeTransfer *> pending;
  std::vector<NativeTransfer *> events;

  LibusbTransferTransport::Api api() {
    LibusbTransferTransport::Api a;
    a.alloc_transfer = [](int) -> void * { return new NativeTransfer{}; };
    a.free_transfer = [](void * p) { delete static_cast<NativeTransfer *>(p); };
    a.fill_bulk_transfer = [](void * p, void *, std::uint8_t, std::uint8_t * buffer,
      int length, unsigned int, std::uint8_t,
      LibusbTransferTransport::NativeCallback cb, void * user) {
      auto * t = static_cast<NativeTransfer *>(p);
      t->buffer = buffer; t->length = length; t->callback = cb; t->user_data = user;
      return 0;
    };
    a.submit_transfer = [this](void * p) {
      std::lock_guard<std::mutex> lock(mutex);
      pending.push_back(static_cast<NativeTransfer *>(p));
      return 0;
    };
    a.cancel_transfer = [this](void * p) {
      std::lock_guard<std::mutex> lock(mutex);
      auto * t = static_cast<NativeTransfer *>(p);
      t->status = 3; t->actual_length = 0;
      events.push_back(t);
      return 0;
    };
    a.handle_events = [this](unsigned int) {
      NativeTransfer * t = nullptr;
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (events.empty()) return 0;
        t = events.front();
        events.erase(events.begin());
        auto it = std::find(pending.begin(), pending.end(), t);
        if (it != pending.end()) pending.erase(it);
      }
      t->callback(t, t->status, t->actual_length, t->user_data);
      return 0;
    };
    a.close_device = [](void *) {};
    return a;
  }
};

int main() {
  FakeUsb fake;
  LibusbTransferTransport transport(fake.api(), reinterpret_cast<void *>(0x1));
  LibusbEventLoop loop(transport, 2);
  assert(loop.start());
  assert(!transport.close());  // Device handle cannot close while event owner is active.

  // start() returns only after the worker has bound the event-loop owner.
  // A second thread cannot pump the same context.
  assert(transport.poll_events(0) < 0);

  std::atomic<int> completed{0};
  LibusbTransferRequest request;
  request.endpoint = 0x81;
  request.direction = UsbTransferDirection::In;
  request.buffer.resize(16);
  assert(transport.submit(7, 4, std::move(request),
    [&](std::uint64_t id, std::uint64_t generation, LibusbCompletionStatus status,
        std::vector<std::uint8_t>, int actual_length) {
      assert(id == 7 && generation == 4);
      assert(status == LibusbCompletionStatus::Cancelled);
      assert(actual_length == 0);
      ++completed;
      LibusbTransferRequest rejected;
      rejected.endpoint = 0x81;
      rejected.direction = UsbTransferDirection::In;
      rejected.buffer.resize(4);
      assert(!transport.submit(8, 4, std::move(rejected), [](auto...) {}));
    }));

  assert(loop.shutdown(1000));
  assert(completed.load() == 1);
  assert(transport.drained());
  assert(transport.pending_count() == 0);
  assert(transport.close());
  return 0;
}
