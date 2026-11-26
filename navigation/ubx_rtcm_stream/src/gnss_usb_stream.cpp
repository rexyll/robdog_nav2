#include "robodog_protocol/gnss_usb_stream.hpp"

#include <limits>
#include <exception>
#include <stdexcept>
#include <utility>

namespace robodog_protocol {

GnssUsbStream::GnssUsbStream(
  LibusbDeviceSession::Api device_api,
  LibusbDeviceRuntime::TransferApiFactory transfer_api_factory,
  GnssUsbStreamConfig config)
: config_(std::move(config)),
  decoder_(config_.max_ubx_payload, config_.max_decoder_buffer_bytes,
           config_.max_nmea_sentence, config_.require_nmea_checksum),
  frame_queue_(config_.frame_queue_capacity, config_.queue_overflow_policy, 0),
  runtime_(std::move(device_api), std::move(transfer_api_factory))
{
  if (config_.read_buffer_bytes == 0 ||
      config_.read_buffer_bytes > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      config_.in_flight_reads == 0) {
    throw std::invalid_argument("invalid GNSS USB read configuration");
  }
}

GnssUsbStream::~GnssUsbStream()
{
  if (runtime_.is_open() && !stop(5000)) std::terminate();
}

bool GnssUsbStream::start()
{
  if (running_.load(std::memory_order_acquire) || runtime_.is_open()) return false;
  if (!runtime_.open(config_.device, config_.event_poll_timeout_ms)) {
    frame_queue_.close();
    return false;
  }

  const auto generation = generation_.fetch_add(1, std::memory_order_acq_rel) + 1;
  frame_queue_.reset_generation(generation);
  {
    std::lock_guard<std::mutex> lock(decoder_mutex_);
    decoder_.reset(generation);
  }
  device_lost_.store(false, std::memory_order_release);
  running_.store(true, std::memory_order_release);

  for (std::size_t i = 0; i < config_.in_flight_reads; ++i) {
    if (!submit_read(generation)) {
      running_.store(false, std::memory_order_release);
      {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++counters_.submit_failures;
      }
      if (!runtime_.shutdown(5000)) return false;
      frame_queue_.close();
      return false;
    }
  }
  return true;
}

bool GnssUsbStream::stop(std::uint32_t timeout_ms)
{
  running_.store(false, std::memory_order_release);
  if (runtime_.is_open() && !runtime_.shutdown(timeout_ms)) return false;
  frame_queue_.close();
  return true;
}

std::optional<Frame> GnssUsbStream::try_pop_frame()
{
  return frame_queue_.try_pop();
}

GnssUsbStreamStats GnssUsbStream::stats() const
{
  GnssUsbStreamStats snapshot;
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    snapshot = counters_;
  }
  snapshot.running = running_.load(std::memory_order_acquire);
  snapshot.device_lost = device_lost_.load(std::memory_order_acquire);
  snapshot.connection_generation = generation_.load(std::memory_order_acquire);
  {
    std::lock_guard<std::mutex> lock(decoder_mutex_);
    snapshot.decoder = decoder_.stats();
  }
  snapshot.queue = frame_queue_.stats();
  return snapshot;
}

bool GnssUsbStream::submit_read(std::uint64_t generation)
{
  if (!running_.load(std::memory_order_acquire)) return false;
  auto * transport = runtime_.transport();
  if (!transport) return false;

  LibusbTransferRequest request;
  request.endpoint = runtime_.device_session().bulk_in().address;
  request.direction = UsbTransferDirection::In;
  request.buffer.resize(config_.read_buffer_bytes);
  request.timeout_ms = config_.transfer_timeout_ms;

  const auto id = next_transfer_id_.fetch_add(1, std::memory_order_relaxed);
  return transport->submit(id, generation, std::move(request),
    [this](std::uint64_t transfer_id, std::uint64_t callback_generation,
           LibusbCompletionStatus status, std::vector<std::uint8_t> bytes,
           int actual_length) {
      on_completion(transfer_id, callback_generation, status,
                    std::move(bytes), actual_length);
    });
}

void GnssUsbStream::on_completion(
  std::uint64_t transfer_id, std::uint64_t generation,
  LibusbCompletionStatus status, std::vector<std::uint8_t> bytes,
  int actual_length)
{
  (void)transfer_id;
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++counters_.transfer_completions;
  }
  if (generation != generation_.load(std::memory_order_acquire)) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++counters_.stale_callbacks;
    return;
  }

  bool resubmit = false;
  switch (status) {
    case LibusbCompletionStatus::Completed:
    case LibusbCompletionStatus::TimedOut: {
      if (actual_length > 0 && !bytes.empty()) {
        std::vector<Frame> frames;
        {
          std::lock_guard<std::mutex> lock(decoder_mutex_);
          frames = decoder_.feed(bytes);
        }
        std::uint64_t decoded = 0;
        std::uint64_t drops = 0;
        std::uint64_t rejections = 0;
        for (auto & frame : frames) {
          ++decoded;
          const auto result = frame_queue_.push(std::move(frame));
          if (result == QueuePushResult::AcceptedAfterDroppingOldest) ++drops;
          else if (result == QueuePushResult::RejectedFull) ++rejections;
          else if (result == QueuePushResult::RejectedStaleGeneration ||
                   result == QueuePushResult::RejectedClosed) ++rejections;
        }
        std::lock_guard<std::mutex> lock(stats_mutex_);
        counters_.decoded_frames += decoded;
        counters_.queue_drops += drops;
        counters_.queue_rejections += rejections;
      }
      resubmit = running_.load(std::memory_order_acquire);
      break;
    }
    case LibusbCompletionStatus::Cancelled: {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      ++counters_.cancelled_transfers;
      break;
    }
    case LibusbCompletionStatus::NoDevice: {
      device_lost_.store(true, std::memory_order_release);
      running_.store(false, std::memory_order_release);
      {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++counters_.no_device_events;
      }
      cancel_remaining_transfers();
      break;
    }
    default: {
      running_.store(false, std::memory_order_release);
      {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++counters_.transport_errors;
      }
      cancel_remaining_transfers();
      break;
    }
  }

  if (resubmit && running_.load(std::memory_order_acquire) && !submit_read(generation)) {
    if (running_.exchange(false, std::memory_order_acq_rel)) {
      {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++counters_.submit_failures;
      }
      cancel_remaining_transfers();
    }
  }
}

void GnssUsbStream::cancel_remaining_transfers()
{
  if (auto * transport = runtime_.transport()) {
    (void)transport->cancel_all_detailed();
  }
}

}  // namespace robodog_protocol
