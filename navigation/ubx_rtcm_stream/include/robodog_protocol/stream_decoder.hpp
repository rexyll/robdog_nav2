#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace robodog_protocol {

enum class Protocol : std::uint8_t { Ubx, Rtcm3 };

struct Frame {
  Protocol protocol;
  std::vector<std::uint8_t> bytes;
  std::uint64_t sequence;
  std::uint64_t connection_generation;
};

struct DecoderStats {
  std::uint64_t input_bytes{0};
  std::uint64_t frames_emitted{0};
  std::uint64_t noise_bytes{0};
  std::uint64_t malformed_headers{0};
  std::uint64_t length_rejects{0};
  std::uint64_t checksum_rejects{0};
  std::uint64_t buffer_limit_drops{0};
};

class StreamDecoder {
 public:
  // max_ubx_payload is a deployment policy, not a UBX protocol limit.
  explicit StreamDecoder(std::size_t max_ubx_payload = 65535,
                         std::size_t max_buffer_bytes = 65543 + 1029);

  std::vector<Frame> feed(const std::uint8_t* data, std::size_t size);
  std::vector<Frame> feed(const std::vector<std::uint8_t>& data) {
    return feed(data.data(), data.size());
  }

  void reset(std::uint64_t connection_generation);
  const DecoderStats& stats() const noexcept { return stats_; }
  std::uint64_t connection_generation() const noexcept {
    return connection_generation_;
  }

  static std::uint32_t crc24q(const std::uint8_t* data, std::size_t size);

 private:
  void drain(std::vector<Frame>& output);
  void discard_prefix(std::size_t count, bool noise);

  std::size_t max_ubx_payload_;
  std::size_t max_buffer_bytes_;
  std::vector<std::uint8_t> buffer_;
  DecoderStats stats_;
  std::uint64_t next_sequence_{0};
  std::uint64_t connection_generation_{0};
};

}  // namespace robodog_protocol
