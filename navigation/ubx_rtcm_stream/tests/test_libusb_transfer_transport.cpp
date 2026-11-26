#include "robodog_protocol/libusb_transfer_transport.hpp"

#include <cassert>
#include <algorithm>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <thread>
#include <atomic>
#include <vector>

using robodog_protocol::LibusbCompletionStatus;
using robodog_protocol::LibusbTransferRequest;
using robodog_protocol::LibusbTransferTransport;
using robodog_protocol::LibusbCancelResult;
using robodog_protocol::UsbTransferDirection;

struct FakeTransfer {
  LibusbTransferTransport::NativeCallback callback{};
  void * user_data{};
  std::uint8_t * buffer{};
  int length{};
  int status{};
  int actual_length{};
  bool pending{false};
};

struct FakeLibusb {
  int submit_result{0};
  int cancel_result{0};
  int close_count{0};
  int free_count{0};
  std::uint8_t last_transfer_flags{0};
  std::function<void()> on_cancel;
  std::deque<FakeTransfer *> events;
  std::vector<FakeTransfer *> allocated;

  LibusbTransferTransport::Api api() {
    LibusbTransferTransport::Api a;
    a.alloc_transfer = [this](int) -> void * { auto * p = new FakeTransfer(); allocated.push_back(p); return p; };
    a.free_transfer = [this](void * p) {
      ++free_count;
      auto * transfer = static_cast<FakeTransfer *>(p);
      allocated.erase(std::remove(allocated.begin(), allocated.end(), transfer), allocated.end());
      delete transfer;
    };
    a.fill_bulk_transfer = [this](void * p, void *, std::uint8_t, std::uint8_t * buffer,
      int length, unsigned int, std::uint8_t flags, LibusbTransferTransport::NativeCallback cb, void * user) {
      last_transfer_flags = flags;
      auto * t = static_cast<FakeTransfer *>(p);
      t->buffer = buffer; t->length = length; t->callback = cb; t->user_data = user;
      return 0;
    };
    a.submit_transfer = [this](void * p) {
      if (submit_result != 0) return submit_result;
      static_cast<FakeTransfer *>(p)->pending = true;
      return 0;
    };
    a.cancel_transfer = [this](void * p) {
      if (on_cancel) on_cancel();
      auto * t = static_cast<FakeTransfer *>(p);
      if (cancel_result != 0) return cancel_result;
      t->status = 3; t->actual_length = 0; events.push_back(t);
      return 0;
    };
    a.handle_events = [this](unsigned int) {
      while (!events.empty()) {
        auto * t = events.front(); events.pop_front();
        if (t->pending) {
          t->pending = false;
          t->callback(t, t->status, t->actual_length, t->user_data);
        }
      }
      return 0;
    };
    a.close_device = [this](void *) { ++close_count; };
    return a;
  }

  void complete_next(int status, int actual) {
    for (auto * t : allocated) {
      if (t->pending) { t->status = status; t->actual_length = actual; events.push_back(t); return; }
    }
    assert(false && "no pending fake transfer");
  }
};

int main() {
  {
    FakeLibusb fake;
    LibusbTransferTransport transport(fake.api(), reinterpret_cast<void *>(0x1));
    LibusbTransferRequest request;
    request.endpoint = 0x82;
    request.direction = UsbTransferDirection::In;
    request.buffer.resize(8);
    bool called = false;
    assert(transport.submit(1, 7, request,
      [&](std::uint64_t id, std::uint64_t generation, LibusbCompletionStatus status,
          std::vector<std::uint8_t> data, int actual) {
        called = true;
        assert(id == 1 && generation == 7);
        assert(status == LibusbCompletionStatus::Completed);
        assert(data.size() == 4 && actual == 4);
      }));
    assert(transport.pending_count() == 1);
    fake.complete_next(0, 4);
    assert(transport.poll_events() == 0);
    assert(called);
    assert(fake.free_count == 1);
    assert(transport.pending_count() == 0);
    assert(transport.close());
    assert(fake.close_count == 1);
  }
  {
    FakeLibusb fake;
    fake.submit_result = -1;
    LibusbTransferTransport transport(fake.api(), reinterpret_cast<void *>(0x1));
    LibusbTransferRequest request;
    request.endpoint = 0x81;
    request.buffer.resize(16);
    assert(!transport.submit(2, 1, request, [](auto, auto, auto, auto, auto) {}));
    assert(transport.pending_count() == 0);
    assert(fake.free_count == 1);
    assert(transport.close());
  }
  {
    FakeLibusb fake;
    fake.cancel_result = -9;
    LibusbTransferTransport transport(fake.api(), reinterpret_cast<void *>(0x1));
    LibusbTransferRequest request;
    request.direction = UsbTransferDirection::Out;
    request.endpoint = 0x02;
    request.buffer = {1, 2, 3};
    request.transfer_flags = 0x01;
    bool called = false;
    assert(transport.submit(3, 2, request,
      [&](auto, auto, LibusbCompletionStatus status, auto, auto) {
        called = true;
        assert(status == LibusbCompletionStatus::NoDevice);
      }));
    assert(fake.last_transfer_flags == 0x01);
    assert(!transport.cancel(3));
    assert(transport.pending_count() == 1);
    // A failed cancel does not free ownership; the eventual NO_DEVICE callback does.
    fake.complete_next(5, 0);
    assert(transport.poll_events() == 0);
    assert(called);
    assert(fake.free_count == 1);
    assert(transport.close());
  }

  {
    FakeLibusb fake;
    LibusbTransferTransport transport(fake.api(), reinterpret_cast<void *>(0x1));
    LibusbTransferRequest request;
    request.direction = UsbTransferDirection::In;
    request.endpoint = 0x83;
    request.buffer.resize(32);
    bool called = false;
    assert(transport.submit(4, 3, request,
      [&](auto, auto, LibusbCompletionStatus status, auto, auto) {
        called = true;
        assert(status == LibusbCompletionStatus::Cancelled);
      }));
    assert(transport.cancel(4));
    // cancel success is asynchronous: transfer ownership remains until callback drain.
    assert(transport.pending_count() == 1);
    assert(!transport.close());
    assert(transport.poll_events() == 0);
    assert(called && transport.pending_count() == 0);
    assert(fake.free_count == 1);
    assert(transport.close());
  }

  {
    // A native actual_length larger than the submitted buffer is rejected.
    FakeLibusb fake;
    LibusbTransferTransport transport(fake.api(), reinterpret_cast<void *>(0x1));
    LibusbTransferRequest request;
    request.endpoint = 0x85;
    request.direction = UsbTransferDirection::In;
    request.buffer.resize(4);
    bool called = false;
    assert(transport.submit(40, 9, request,
      [&](auto, auto, LibusbCompletionStatus status, auto data, int actual) {
        called = true;
        assert(status == LibusbCompletionStatus::Overflow);
        assert(data.empty() && actual == 0);
      }));
    fake.complete_next(0, 5);
    assert(transport.poll_events() == 0);
    assert(called && transport.pending_count() == 0);
    assert(transport.close());
  }

  {
    // A negative native actual_length is rejected as malformed metadata.
    FakeLibusb fake;
    LibusbTransferTransport transport(fake.api(), reinterpret_cast<void *>(0x1));
    LibusbTransferRequest request;
    request.endpoint = 0x86;
    request.direction = UsbTransferDirection::In;
    request.buffer.resize(4);
    bool called = false;
    assert(transport.submit(41, 9, request,
      [&](auto, auto, LibusbCompletionStatus status, auto data, int actual) {
        called = true;
        assert(status == LibusbCompletionStatus::Overflow);
        assert(data.empty() && actual == 0);
      }));
    fake.complete_next(0, -1);
    assert(transport.poll_events() == 0);
    assert(called && transport.pending_count() == 0);
    assert(transport.close());
  }

  {
    FakeLibusb fake;
    LibusbTransferTransport transport(fake.api(), reinterpret_cast<void *>(0x1));
    LibusbTransferRequest request;
    request.endpoint = 0x84;
    request.direction = UsbTransferDirection::In;
    request.buffer.resize(4);
    bool second_submitted = false;
    bool close_rejected_during_callback = false;
    assert(transport.submit(5, 8, request,
      [&](auto, auto, auto, auto, auto) {
        close_rejected_during_callback = !transport.close();
        second_submitted = transport.submit(6, 8, request,
          [](auto, auto, auto, auto, auto) {});
      }));
    fake.complete_next(0, 4);
    assert(transport.poll_events() == 0);
    assert(close_rejected_during_callback && second_submitted);
    assert(transport.pending_count() == 1);
    fake.complete_next(0, 4);
    assert(transport.poll_events() == 0);
    assert(transport.pending_count() == 0);
    assert(transport.close());
  }

  {
    // Concurrent pending_count() must be synchronized with cancel()'s
    // registry lookup while the operation lock serializes native libusb calls.
    FakeLibusb fake;
    LibusbTransferTransport transport(fake.api(), reinterpret_cast<void *>(0x1));
    LibusbTransferRequest request;
    request.endpoint = 0x87;
    request.direction = UsbTransferDirection::In;
    request.buffer.resize(8);
    assert(transport.submit(50, 11, request, [](auto, auto, auto, auto, auto) {}));
    std::atomic<bool> start{false};
    std::thread observer([&] {
      while (!start.load(std::memory_order_acquire)) {}
      for (int i = 0; i < 20000; ++i) {
        (void)transport.pending_count();
      }
    });
    // The fake native API calls back into the independent registry observer.
    // This would deadlock if cancel() retained registry_mutex_ across the
    // native libusb_cancel_transfer() call.
    std::size_t observed_during_native_cancel = 0;
    fake.on_cancel = [&] { observed_during_native_cancel = transport.pending_count(); };
    start.store(true, std::memory_order_release);
    assert(transport.cancel(50));
    observer.join();
    assert(observed_during_native_cancel == 1);
    assert(transport.pending_count() == 1);
    assert(transport.poll_events() == 0);
    assert(transport.pending_count() == 0);
    assert(transport.close());
  }

  {
    FakeLibusb fake;
    LibusbTransferTransport transport(fake.api(), reinterpret_cast<void *>(0x1));
    LibusbTransferRequest request;
    request.endpoint = 0x89;
    request.direction = UsbTransferDirection::In;
    request.buffer.resize(8);
    assert(transport.submit(60, 1, request, [](auto, auto, auto, auto, auto) {}));
    fake.cancel_result = -5;
    assert(transport.cancel_detailed(60) == LibusbCancelResult::NotFound);
    assert(transport.pending_count() == 1);
    fake.cancel_result = -4;
    assert(transport.cancel_detailed(60) == LibusbCancelResult::DeviceLost);
    assert(transport.pending_count() == 1);
    fake.complete_next(5, 0);
    assert(transport.poll_events() == 0);
    assert(transport.pending_count() == 0);
    assert(transport.close());
  }

  return 0;
}
