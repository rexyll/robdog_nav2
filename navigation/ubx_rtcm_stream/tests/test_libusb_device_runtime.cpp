#include "robodog_protocol/libusb_device_runtime.hpp"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

using robodog_protocol::LibusbDeviceRuntime;
using robodog_protocol::LibusbTransferTransport;
using robodog_protocol::LibusbTransferRequest;
using robodog_protocol::LibusbCompletionStatus;
using robodog_protocol::UsbBulkEndpointDescriptor;
using robodog_protocol::UsbDeviceSessionConfig;
using robodog_protocol::UsbTransferDirection;

struct RuntimeNativeTransfer {
  LibusbTransferTransport::NativeCallback callback{};
  void * user_data{};
  int status{};
  int actual_length{};
};

struct RuntimeFake {
  std::mutex mutex;
  std::vector<RuntimeNativeTransfer *> pending;
  std::vector<RuntimeNativeTransfer *> events;
  std::atomic<int> callbacks_completed{0};
  bool session_device_closed_after_callback_drain{false};
  int context_exits{0};

  robodog_protocol::LibusbDeviceSession::Api device_api() {
    robodog_protocol::LibusbDeviceSession::Api a;
    a.init_context = [] { return reinterpret_cast<void *>(0x10); };
    a.exit_context = [this](void *) { ++context_exits; };
    a.open_device = [](void *, std::uint16_t, std::uint16_t, const std::string &) {
      return reinterpret_cast<void *>(0x20);
    };
    a.close_device = [this](void *) {
      session_device_closed_after_callback_drain = callbacks_completed.load() == 1;
    };
    a.get_bulk_endpoints = [](void *, std::vector<UsbBulkEndpointDescriptor> & out) {
      out = {{1, 0, 0x81, 64}, {1, 0, 0x02, 64}};
      return true;
    };
    a.kernel_driver_active = [](void *, int) { return 0; };
    a.detach_kernel_driver = [](void *, int) { return 0; };
    a.attach_kernel_driver = [](void *, int) { return 0; };
    a.claim_interface = [](void *, int) { return 0; };
    a.release_interface = [](void *, int) { return 0; };
    a.set_alternate_setting = [](void *, int, int) { return 0; };
    return a;
  }

  LibusbTransferTransport::Api transfer_api(void *) {
    LibusbTransferTransport::Api a;
    a.alloc_transfer = [](int) -> void * { return new RuntimeNativeTransfer{}; };
    a.free_transfer = [](void * p) { delete static_cast<RuntimeNativeTransfer *>(p); };
    a.fill_bulk_transfer = [](void * p, void *, std::uint8_t, std::uint8_t *,
      int, unsigned int, std::uint8_t,
      LibusbTransferTransport::NativeCallback cb, void * user) {
      auto * t = static_cast<RuntimeNativeTransfer *>(p);
      t->callback = cb; t->user_data = user;
      return 0;
    };
    a.submit_transfer = [this](void * p) {
      std::lock_guard<std::mutex> lock(mutex);
      pending.push_back(static_cast<RuntimeNativeTransfer *>(p));
      return 0;
    };
    a.cancel_transfer = [this](void * p) {
      std::lock_guard<std::mutex> lock(mutex);
      auto * t = static_cast<RuntimeNativeTransfer *>(p);
      t->status = 3; t->actual_length = 0;
      events.push_back(t);
      return 0;
    };
    a.handle_events = [this](unsigned int) {
      RuntimeNativeTransfer * t = nullptr;
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
  RuntimeFake fake;
  LibusbDeviceRuntime runtime(fake.device_api(),
    [&fake](void * context) { return fake.transfer_api(context); });
  UsbDeviceSessionConfig config{0x1546, 0x01a9, ""};
  assert(runtime.open(config, 2));
  assert(runtime.is_open());
  assert(runtime.transport() != nullptr);

  std::atomic<int> callback_count{0};
  LibusbTransferRequest request;
  request.endpoint = 0x81;
  request.direction = UsbTransferDirection::In;
  request.buffer.resize(32);
  assert(runtime.transport()->submit(9, 1, std::move(request),
    [&](std::uint64_t, std::uint64_t, LibusbCompletionStatus status,
        std::vector<std::uint8_t>, int) {
      assert(status == LibusbCompletionStatus::Cancelled);
      ++callback_count;
      ++fake.callbacks_completed;
    }));

  assert(runtime.shutdown(1000));
  assert(callback_count.load() == 1);
  assert(fake.session_device_closed_after_callback_drain);
  assert(fake.context_exits == 1);
  assert(!runtime.is_open());
  return 0;
}
