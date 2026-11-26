#include "robodog_protocol/libusb_transfer_transport.hpp"

#include <cassert>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <vector>

using robodog_protocol::LibusbCompletionStatus;
using robodog_protocol::LibusbTransferRequest;
using robodog_protocol::LibusbTransferTransport;
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
  std::deque<FakeTransfer *> events;
  std::vector<FakeTransfer *> allocated;

  LibusbTransferTransport::Api api() {
    LibusbTransferTransport::Api a;
    a.alloc_transfer = [this](int) -> void * { auto * p = new FakeTransfer(); allocated.push_back(p); return p; };
    a.free_transfer = [this](void * p) {
      ++free_count;
      delete static_cast<FakeTransfer *>(p);
    };
    a.fill_bulk_transfer = [](void * p, void *, std::uint8_t, std::uint8_t * buffer,
      int length, unsigned int, LibusbTransferTransport::NativeCallback cb, void * user) {
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
    request.endpoint = 2;
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
    request.buffer = {1, 2, 3};
    bool called = false;
    assert(transport.submit(3, 2, request,
      [&](auto, auto, LibusbCompletionStatus status, auto, auto) {
        called = true;
        assert(status == LibusbCompletionStatus::NoDevice);
      }));
    assert(!transport.cancel(3));
    assert(transport.pending_count() == 1);
    // A failed cancel does not free ownership; the eventual NO_DEVICE callback does.
    fake.complete_next(5, 0);
    assert(transport.poll_events() == 0);
    assert(called);
    assert(fake.free_count == 1);
    assert(transport.close());
  }
  return 0;
}
