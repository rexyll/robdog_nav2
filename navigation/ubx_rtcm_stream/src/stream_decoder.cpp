#include "robodog_protocol/stream_decoder.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace robodog_protocol {
namespace {
constexpr std::uint8_t kUbxSync1 = 0xB5;
constexpr std::uint8_t kUbxSync2 = 0x62;
constexpr std::uint8_t kRtcmPreamble = 0xD3;
constexpr std::uint8_t kNmeaStart = '$';
constexpr std::uint8_t kCarriageReturn = '\r';
constexpr std::uint8_t kLineFeed = '\n';

bool ubx_checksum_valid(const std::vector<std::uint8_t>& bytes,
                        std::size_t payload_length) {
  std::uint8_t ck_a = 0;
  std::uint8_t ck_b = 0;
  const std::size_t end = 6 + payload_length;
  for (std::size_t i = 2; i < end; ++i) {
    ck_a = static_cast<std::uint8_t>(ck_a + bytes[i]);
    ck_b = static_cast<std::uint8_t>(ck_b + ck_a);
  }
  return ck_a == bytes[end] && ck_b == bytes[end + 1];
}

int hex_value(std::uint8_t value) {
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'A' && value <= 'F') return value - 'A' + 10;
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  return -1;
}

ChecksumStatus nmea_checksum_status(const std::vector<std::uint8_t>& bytes,
                                    std::size_t sentence_length) {
  const auto end = bytes.begin() + static_cast<std::ptrdiff_t>(sentence_length);
  const auto star = std::find(bytes.begin(), end, static_cast<std::uint8_t>('*'));
  if (star == end) return ChecksumStatus::Absent;

  const auto star_index = static_cast<std::size_t>(star - bytes.begin());
  if (star_index + 3 != sentence_length) return ChecksumStatus::NotApplicable;
  const int high = hex_value(bytes[star_index + 1]);
  const int low = hex_value(bytes[star_index + 2]);
  if (high < 0 || low < 0) return ChecksumStatus::NotApplicable;

  std::uint8_t calculated = 0;
  for (std::size_t i = 1; i < star_index; ++i) calculated ^= bytes[i];
  const auto received = static_cast<std::uint8_t>((high << 4) | low);
  return calculated == received ? ChecksumStatus::Valid
                                : ChecksumStatus::NotApplicable;
}
}  // namespace

StreamDecoder::StreamDecoder(std::size_t max_ubx_payload,
                             std::size_t max_buffer_bytes,
                             std::size_t max_nmea_sentence,
                             bool require_nmea_checksum)
    : max_ubx_payload_(max_ubx_payload),
      max_buffer_bytes_(max_buffer_bytes),
      max_nmea_sentence_(max_nmea_sentence),
      require_nmea_checksum_(require_nmea_checksum) {
  if (max_ubx_payload_ > 65535) {
    throw std::invalid_argument("UBX payload ceiling exceeds uint16 protocol field");
  }
  if (max_nmea_sentence_ < 6) {
    throw std::invalid_argument("NMEA sentence ceiling is too small");
  }
  if (max_buffer_bytes_ < max_ubx_payload_ + 8 ||
      max_buffer_bytes_ < 1029 ||
      max_buffer_bytes_ < max_nmea_sentence_ + 1) {
    throw std::invalid_argument("buffer limit cannot hold the largest accepted frame");
  }
  buffer_.reserve(std::min<std::size_t>(max_buffer_bytes_, 65543 + 1029));
}

std::uint32_t StreamDecoder::crc24q(const std::uint8_t* data,
                                    std::size_t size) {
  std::uint32_t crc = 0;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<std::uint32_t>(data[i]) << 16;
    for (int bit = 0; bit < 8; ++bit) {
      crc <<= 1;
      if ((crc & 0x1000000U) != 0) crc ^= 0x1864CFBU;
    }
  }
  return crc & 0xFFFFFFU;
}

void StreamDecoder::discard_prefix(std::size_t count, bool noise) {
  if (count == 0) return;
  if (noise) stats_.noise_bytes += count;
  buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(count));
}

void StreamDecoder::drain(std::vector<Frame>& output) {
  for (;;) {
    if (buffer_.empty()) return;

    if (buffer_[0] == kUbxSync1) {
      if (buffer_.size() == 1) return;
      if (buffer_[1] == kUbxSync2) {
        if (buffer_.size() < 6) return;
        const std::size_t payload_length =
            static_cast<std::size_t>(buffer_[4]) |
            (static_cast<std::size_t>(buffer_[5]) << 8);
        if (payload_length > max_ubx_payload_) {
          ++stats_.length_rejects;
          ++stats_.malformed_headers;
          discard_prefix(1, false);
          continue;
        }
        const std::size_t frame_length = payload_length + 8;
        if (buffer_.size() < frame_length) return;
        if (!ubx_checksum_valid(buffer_, payload_length)) {
          ++stats_.checksum_rejects;
          discard_prefix(1, false);
          continue;
        }
        Frame frame{Protocol::Ubx,
                    std::vector<std::uint8_t>(buffer_.begin(),
                      buffer_.begin() + static_cast<std::ptrdiff_t>(frame_length)),
                    next_sequence_++, connection_generation_};
        output.push_back(std::move(frame));
        ++stats_.frames_emitted;
        discard_prefix(frame_length, false);
        continue;
      }
    }

    if (buffer_[0] == kRtcmPreamble) {
      if (buffer_.size() < 2) return;
      if ((buffer_[1] & 0xFCU) != 0) {
        ++stats_.malformed_headers;
        discard_prefix(1, false);
        continue;
      }
      if (buffer_.size() < 3) return;
      const std::size_t payload_length =
          (static_cast<std::size_t>(buffer_[1] & 0x03U) << 8) |
          static_cast<std::size_t>(buffer_[2]);
      const std::size_t frame_length = payload_length + 6;
      if (buffer_.size() < frame_length) return;
      const std::uint32_t expected =
          (static_cast<std::uint32_t>(buffer_[frame_length - 3]) << 16) |
          (static_cast<std::uint32_t>(buffer_[frame_length - 2]) << 8) |
          static_cast<std::uint32_t>(buffer_[frame_length - 1]);
      if (crc24q(buffer_.data(), frame_length - 3) != expected) {
        ++stats_.checksum_rejects;
        discard_prefix(1, false);
        continue;
      }
      Frame frame{Protocol::Rtcm3,
                  std::vector<std::uint8_t>(buffer_.begin(),
                    buffer_.begin() + static_cast<std::ptrdiff_t>(frame_length)),
                  next_sequence_++, connection_generation_};
      output.push_back(std::move(frame));
      ++stats_.frames_emitted;
      discard_prefix(frame_length, false);
      continue;
    }

    if (buffer_[0] == kNmeaStart) {
      const auto terminator = std::find_if(buffer_.begin(), buffer_.end(),
          [](std::uint8_t value) {
            return value == kCarriageReturn || value == kLineFeed;
          });
      const bool has_terminator = terminator != buffer_.end();
      const std::size_t sentence_length = has_terminator
          ? static_cast<std::size_t>(terminator - buffer_.begin())
          : buffer_.size();
      if (sentence_length > max_nmea_sentence_) {
        ++stats_.length_rejects;
        ++stats_.malformed_headers;
        discard_prefix(1, false);
        continue;
      }
      if (!has_terminator) return;

      const auto checksum = nmea_checksum_status(buffer_, sentence_length);
      if (checksum == ChecksumStatus::NotApplicable ||
          (require_nmea_checksum_ && checksum == ChecksumStatus::Absent)) {
        ++stats_.checksum_rejects;
        discard_prefix(1, false);
        continue;
      }

      Frame frame{Protocol::Nmea,
                  std::vector<std::uint8_t>(buffer_.begin(),
                    buffer_.begin() + static_cast<std::ptrdiff_t>(sentence_length)),
                  next_sequence_++, connection_generation_, checksum};
      output.push_back(std::move(frame));
      ++stats_.frames_emitted;
      std::size_t consumed = sentence_length + 1;
      if (buffer_[sentence_length] == kCarriageReturn &&
          buffer_.size() > consumed && buffer_[consumed] == kLineFeed) {
        ++consumed;
      }
      discard_prefix(consumed, false);
      continue;
    }

    discard_prefix(1, true);
  }
}

std::vector<Frame> StreamDecoder::feed(const std::uint8_t* data,
                                       std::size_t size) {
  if (size != 0 && data == nullptr) {
    throw std::invalid_argument("non-empty input has a null data pointer");
  }
  std::vector<Frame> output;
  stats_.input_bytes += size;
  for (std::size_t i = 0; i < size; ++i) {
    if (buffer_.size() >= max_buffer_bytes_) {
      ++stats_.buffer_limit_drops;
      discard_prefix(1, true);
      drain(output);
    }
    buffer_.push_back(data[i]);
    drain(output);
  }
  return output;
}

void StreamDecoder::reset(std::uint64_t connection_generation) {
  buffer_.clear();
  connection_generation_ = connection_generation;
}

}  // namespace robodog_protocol
