#include "robodog_protocol/gnss_usb_stream.hpp"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <optional>
#include <utility>
#include <vector>

using robodog_protocol::GnssUsbStream;
using robodog_protocol::GnssUsbStreamConfig;
using robodog_protocol::LibusbTransferTransport;
using robodog_protocol::LibusbCompletionStatus;
using robodog_protocol::UsbBulkEndpointDescriptor;
using robodog_protocol::UsbDeviceSessionConfig;
using robodog_protocol::Protocol;

struct GnssNativeTransfer {
  LibusbTransferTransport::NativeCallback callback{};
  void * user_data{};
  std::uint8_t * buffer{};
  int length{};
  int status{};
  int actual_length{};
};

struct FakeGnssUsb {
  std::mutex mutex;
  std::vector<GnssNativeTransfer *> pending;
  std::vector<GnssNativeTransfer *> events;
  int context_exits{0};
  int device_closes{0};

  robodog_protocol::LibusbDeviceSession::Api device_api() {
    robodog_protocol::LibusbDeviceSession::Api a;
    a.init_context = [] { return reinterpret_cast<void *>(0x10); };
    a.exit_context = [this](void *) { ++context_exits; };
    a.open_device = [](void *, std::uint16_t, std::uint16_t, const std::string &) {
      return reinterpret_cast<void *>(0x20);
    };
    a.close_device = [this](void *) { ++device_closes; };
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
    a.alloc_transfer = [](int) -> void * { return new GnssNativeTransfer{}; };
    a.free_transfer = [](void * p) { delete static_cast<GnssNativeTransfer *>(p); };
    a.fill_bulk_transfer = [](void * p, void *, std::uint8_t, std::uint8_t * buffer,
      int length, unsigned int, std::uint8_t,
      LibusbTransferTransport::NativeCallback cb, void * user) {
      auto * t = static_cast<GnssNativeTransfer *>(p);
      t->buffer = buffer; t->length = length; t->callback = cb; t->user_data = user;
      return 0;
    };
    a.submit_transfer = [this](void * p) {
      std::lock_guard<std::mutex> lock(mutex);
      pending.push_back(static_cast<GnssNativeTransfer *>(p));
      return 0;
    };
    a.cancel_transfer = [this](void * p) {
      std::lock_guard<std::mutex> lock(mutex);
      auto * t = static_cast<GnssNativeTransfer *>(p);
      t->status = 3; t->actual_length = 0;
      events.push_back(t);
      return 0;
    };
    a.handle_events = [this](unsigned int) {
      GnssNativeTransfer * t = nullptr;
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

  bool inject(const std::vector<std::uint8_t> & bytes) {
    std::lock_guard<std::mutex> lock(mutex);
    if (pending.empty()) return false;
    auto * t = pending.front();
    if (bytes.size() > static_cast<std::size_t>(t->length)) return false;
    std::memcpy(t->buffer, bytes.data(), bytes.size());
    t->status = 0;
    t->actual_length = static_cast<int>(bytes.size());
    events.push_back(t);
    return true;
  }

  std::size_t pending_count() {
    std::lock_guard<std::mutex> lock(mutex);
    return pending.size();
  }
};

int main() {
  FakeGnssUsb fake;
  GnssUsbStreamConfig config;
  config.device = UsbDeviceSessionConfig{0x1546, 0x01a9, ""};
  config.process_session_id = "gnss-usb-stream-test-session";
  config.read_buffer_bytes = 128;
  config.in_flight_reads = 2;
  config.frame_queue_capacity = 8;
  config.event_poll_timeout_ms = 2;

  GnssUsbStream stream(fake.device_api(),
    [&fake](void * context) { return fake.transfer_api(context); }, config);
  assert(stream.start());
  assert(fake.pending_count() == 2);

  auto started = stream.try_pop_event();
  assert(started.has_value());
  assert(started->type == robodog_protocol::GnssIngressEventType::GenerationStarted);
  assert(started->connection_generation == 1);
  assert(started->process_session_id == "gnss-usb-stream-test-session");

  // Generic zero-payload UBX frame with a valid Fletcher checksum.
  const std::vector<std::uint8_t> ubx{0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19};
  assert(fake.inject(ubx));

  std::optional<robodog_protocol::GnssIngressEvent> frame_event;
  for (int i = 0; i < 200 && !frame_event; ++i) {
    frame_event = stream.try_pop_event();
    if (!frame_event) std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  assert(frame_event.has_value());
  assert(frame_event->type == robodog_protocol::GnssIngressEventType::Frame);
  assert(frame_event->frame.has_value());
  assert(frame_event->frame->protocol == Protocol::Ubx);
  assert(frame_event->frame->connection_generation == 1);
  assert(frame_event->process_session_id == "gnss-usb-stream-test-session");
  assert(frame_event->ingress_sequence > started->ingress_sequence);
  assert(frame_event->receive_monotonic_ns >= started->receive_monotonic_ns);
  for (int i = 0; i < 100 && fake.pending_count() != 2; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  assert(fake.pending_count() == 2);  // completion resubmitted one IN transfer

  const auto stats = stream.stats();
  assert(stats.decoded_frames == 1);
  assert(stats.transfer_completions >= 1);
  assert(stream.stop(1000));
  assert(!stream.is_running());
  assert(fake.pending_count() == 0);
  assert(fake.device_closes == 1);
  assert(fake.context_exits == 1);
  auto stopped = stream.try_pop_event();
  assert(stopped.has_value());
  assert(stopped->type == robodog_protocol::GnssIngressEventType::Stopped);
  assert(stopped->ingress_sequence > frame_event->ingress_sequence);

  // Restart the same stream object. The lifecycle queue must preserve its
  // global sequence while the connection generation advances.
  assert(stream.start());
  assert(fake.pending_count() == 2);
  auto started2 = stream.try_pop_event();
  assert(started2.has_value());
  assert(started2->type == robodog_protocol::GnssIngressEventType::GenerationStarted);
  assert(started2->connection_generation == 2);
  assert(started2->ingress_sequence > stopped->ingress_sequence);

  assert(fake.inject(ubx));
  std::optional<robodog_protocol::GnssIngressEvent> frame_event2;
  for (int i = 0; i < 200 && !frame_event2; ++i) {
    frame_event2 = stream.try_pop_event();
    if (!frame_event2) std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  assert(frame_event2.has_value());
  assert(frame_event2->type == robodog_protocol::GnssIngressEventType::Frame);
  assert(frame_event2->connection_generation == 2);
  assert(frame_event2->frame && frame_event2->frame->connection_generation == 2);
  assert(frame_event2->ingress_sequence > started2->ingress_sequence);
  assert(stream.stop(1000));
  auto stopped2 = stream.try_pop_event();
  assert(stopped2.has_value());
  assert(stopped2->type == robodog_protocol::GnssIngressEventType::Stopped);
  assert(stopped2->connection_generation == 2);
  assert(stopped2->ingress_sequence > frame_event2->ingress_sequence);
  assert(fake.device_closes == 2);
  assert(fake.context_exits == 2);
  return 0;
}
