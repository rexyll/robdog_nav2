#pragma once

#include "robodog_protocol/bounded_frame_queue.hpp"
#include "robodog_protocol/libusb_device_runtime.hpp"
#include "robodog_protocol/gnss_ingress_queue.hpp"
#include "robodog_protocol/stream_decoder.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace robodog_protocol {

struct GnssUsbStreamConfig {
  UsbDeviceSessionConfig device;
  std::string process_session_id;
  std::size_t read_buffer_bytes{12800};
  std::size_t in_flight_reads{4};
  std::size_t frame_queue_capacity{256};
  OverflowPolicy queue_overflow_policy{OverflowPolicy::DropOldest};
  std::size_t max_ubx_payload{65535};
  std::size_t max_decoder_buffer_bytes{66572};
  std::size_t max_nmea_sentence{1024};
  bool require_nmea_checksum{false};
  unsigned int transfer_timeout_ms{0};
  unsigned int event_poll_timeout_ms{50};
};

struct GnssUsbStreamStats {
  bool running{false};
  bool device_lost{false};
  std::uint64_t connection_generation{0};
  std::uint64_t transfer_completions{0};
  std::uint64_t cancelled_transfers{0};
  std::uint64_t transport_errors{0};
  std::uint64_t no_device_events{0};
  std::uint64_t submit_failures{0};
  std::uint64_t stale_callbacks{0};
  std::uint64_t decoded_frames{0};
  std::uint64_t queue_drops{0};
  std::uint64_t queue_rejections{0};
  DecoderStats decoder;
  QueueStats queue;
  GnssIngressQueueStats ingress;
};

// Bridges asynchronous USB bulk-IN reads into the incremental UBX/RTCM3/NMEA
// decoder and a bounded, generation-fenced frame queue.
class GnssUsbStream {
public:
  GnssUsbStream(
    LibusbDeviceSession::Api device_api,
    LibusbDeviceRuntime::TransferApiFactory transfer_api_factory,
    GnssUsbStreamConfig config);
  GnssUsbStream(const GnssUsbStream &) = delete;
  GnssUsbStream & operator=(const GnssUsbStream &) = delete;
  ~GnssUsbStream();

  bool start();
  bool stop(std::uint32_t timeout_ms = 5000);
  // Legacy frame-only API: consumes and discards lifecycle control events.
  // Generation-aware consumers must use try_pop_event().
  std::optional<Frame> try_pop_frame();
  std::optional<GnssIngressEvent> try_pop_event();
  GnssUsbStreamStats stats() const;
  bool is_running() const noexcept { return running_.load(std::memory_order_acquire); }

private:
  bool submit_read(std::uint64_t generation);
  void on_completion(
    std::uint64_t transfer_id, std::uint64_t generation,
    LibusbCompletionStatus status, std::vector<std::uint8_t> bytes,
    int actual_length);
  void cancel_remaining_transfers();

  GnssUsbStreamConfig config_;
  StreamDecoder decoder_;
  GnssIngressQueue ingress_queue_;
  std::atomic<bool> running_{false};
  std::atomic<bool> device_lost_{false};
  std::atomic<std::uint64_t> generation_{0};
  std::atomic<std::uint64_t> next_transfer_id_{1};
  mutable std::mutex lifecycle_mutex_;
  mutable std::mutex decoder_mutex_;
  mutable std::mutex stats_mutex_;
  GnssUsbStreamStats counters_;
  // Declared last so it is destroyed first; its shutdown drains callbacks
  // while decoder_, frame_queue_, and counters_ are still alive.
  LibusbDeviceRuntime runtime_;
};

}  // namespace robodog_protocol
