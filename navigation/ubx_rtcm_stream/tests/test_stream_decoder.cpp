#include "robodog_protocol/stream_decoder.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using robodog_protocol::ChecksumStatus;
using robodog_protocol::Frame;
using robodog_protocol::Protocol;
using robodog_protocol::StreamDecoder;
using Bytes = std::vector<std::uint8_t>;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

Bytes make_ubx(Bytes payload, std::uint8_t cls = 0x01, std::uint8_t id = 0x07) {
  const auto n = static_cast<std::uint16_t>(payload.size());
  Bytes out{0xB5, 0x62, cls, id, static_cast<std::uint8_t>(n & 0xFF),
            static_cast<std::uint8_t>(n >> 8)};
  out.insert(out.end(), payload.begin(), payload.end());
  std::uint8_t a = 0, b = 0;
  for (std::size_t i = 2; i < out.size(); ++i) {
    a = static_cast<std::uint8_t>(a + out[i]);
    b = static_cast<std::uint8_t>(b + a);
  }
  out.push_back(a);
  out.push_back(b);
  return out;
}

Bytes make_rtcm(std::size_t payload_size) {
  Bytes out{0xD3, static_cast<std::uint8_t>((payload_size >> 8) & 0x03),
            static_cast<std::uint8_t>(payload_size & 0xFF)};
  for (std::size_t i = 0; i < payload_size; ++i)
    out.push_back(static_cast<std::uint8_t>((i * 37U + 11U) & 0xFFU));
  const auto crc = StreamDecoder::crc24q(out.data(), out.size());
  out.push_back(static_cast<std::uint8_t>((crc >> 16) & 0xFF));
  out.push_back(static_cast<std::uint8_t>((crc >> 8) & 0xFF));
  out.push_back(static_cast<std::uint8_t>(crc & 0xFF));
  return out;
}

Bytes make_nmea(const std::string& body, bool with_checksum = true) {
  Bytes out{'$'};
  std::uint8_t checksum = 0;
  for (const auto ch : body) {
    out.push_back(static_cast<std::uint8_t>(ch));
    checksum ^= static_cast<std::uint8_t>(ch);
  }
  if (with_checksum) {
    constexpr char hex[] = "0123456789ABCDEF";
    out.push_back('*');
    out.push_back(static_cast<std::uint8_t>(hex[(checksum >> 4) & 0x0F]));
    out.push_back(static_cast<std::uint8_t>(hex[checksum & 0x0F]));
  }
  return out;
}

std::vector<Frame> decode_chunks(const Bytes& stream, std::uint32_t seed) {
  StreamDecoder decoder;
  std::mt19937 random(seed);
  std::vector<Frame> result;
  std::size_t offset = 0;
  while (offset < stream.size()) {
    const auto chunk = std::min<std::size_t>(
        1 + (random() % 17), stream.size() - offset);
    auto frames = decoder.feed(stream.data() + offset, chunk);
    result.insert(result.end(), frames.begin(), frames.end());
    offset += chunk;
  }
  return result;
}

void require_same_frames(const std::vector<Frame>& a, const std::vector<Frame>& b) {
  require(a.size() == b.size(), "property test frame count mismatch");
  for (std::size_t i = 0; i < a.size(); ++i) {
    require(a[i].protocol == b[i].protocol && a[i].bytes == b[i].bytes &&
            a[i].sequence == b[i].sequence &&
            a[i].connection_generation == b[i].connection_generation &&
            a[i].checksum_status == b[i].checksum_status,
            "property test frame mismatch at index " + std::to_string(i));
  }
}

void test_crc24q_check_value() {
  const std::string s = "123456789";
  require(StreamDecoder::crc24q(reinterpret_cast<const std::uint8_t*>(s.data()),
                                s.size()) == 0xCDE703U,
          "CRC24Q check value mismatch");
}

void test_ubx_fragmentation_and_metadata() {
  const auto frame = make_ubx({0x10, 0x20, 0x30});
  for (std::size_t split = 0; split <= frame.size(); ++split) {
    StreamDecoder decoder;
    auto a = decoder.feed(frame.data(), split);
    auto b = decoder.feed(frame.data() + split, frame.size() - split);
    const auto& emitted = split == frame.size() ? a : b;
    require((split == frame.size() ? b.empty() : a.empty()) && emitted.size() == 1 &&
            emitted[0].protocol == Protocol::Ubx && emitted[0].bytes == frame,
            "UBX split frame failed");
  }
}

void test_multiple_frames_noise_and_recovery() {
  auto u = make_ubx({1, 2, 3});
  auto r = make_rtcm(256);
  auto n = make_nmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
  Bytes stream{0x00, 0x44, 0x99};
  stream.insert(stream.end(), u.begin(), u.end());
  stream.push_back(0x13);
  stream.insert(stream.end(), r.begin(), r.end());
  stream.insert(stream.end(), n.begin(), n.end());
  stream.push_back('\r');
  stream.push_back('\n');
  auto bad = make_ubx({4, 5});
  bad[bad.size() - 2] ^= 0x01;
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.insert(stream.end(), u.begin(), u.end());

  StreamDecoder whole;
  const auto expected = whole.feed(stream);
  require(expected.size() == 4, "single-chunk mixed frame decode failed");
  require(expected[2].protocol == Protocol::Nmea &&
          expected[2].checksum_status == ChecksumStatus::Valid,
          "NMEA valid checksum status missing");

  for (std::uint32_t seed = 0; seed < 100; ++seed) {
    const auto actual = decode_chunks(stream, seed);
    require_same_frames(expected, actual);
  }
}

void test_ubx_checksum_bytes_independently() {
  const auto good = make_ubx({0xAA, 0x55});
  for (const auto index : {good.size() - 2, good.size() - 1}) {
    auto bad = good;
    bad[index] ^= 0x80;
    StreamDecoder decoder;
    require(decoder.feed(bad).empty(), "corrupt UBX checksum accepted");
    require(decoder.stats().checksum_rejects == 1, "UBX checksum rejection not counted");
  }
}

void test_rtcm_length_boundaries() {
  for (const std::size_t length : {0U, 1U, 255U, 256U, 511U, 1023U}) {
    const auto frame = make_rtcm(length);
    StreamDecoder decoder;
    std::vector<Frame> frames;
    for (std::size_t i = 0; i < frame.size(); i += 7) {
      const auto count = std::min<std::size_t>(7, frame.size() - i);
      auto part = decoder.feed(frame.data() + i, count);
      frames.insert(frames.end(), part.begin(), part.end());
    }
    require(frames.size() == 1 && frames[0].protocol == Protocol::Rtcm3 &&
            frames[0].bytes == frame, "RTCM boundary failed: " + std::to_string(length));
  }
}

void test_rtcm_reserved_and_crc_rejection() {
  auto reserved = make_rtcm(2);
  reserved[1] |= 0x04;
  auto corrupt = make_rtcm(12);
  corrupt.back() ^= 0x01;
  auto valid = make_rtcm(1);
  Bytes input = reserved;
  input.insert(input.end(), corrupt.begin(), corrupt.end());
  input.insert(input.end(), valid.begin(), valid.end());
  StreamDecoder decoder;
  const auto frames = decoder.feed(input);
  require(frames.size() == 1 && frames[0].bytes == valid,
          "RTCM malformed frame recovery failed");
  require(decoder.stats().malformed_headers >= 1 &&
          decoder.stats().checksum_rejects >= 1,
          "RTCM rejects not counted");
}

void test_nmea_checksum_and_terminators() {
  auto valid = make_nmea("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W");
  Bytes input = valid;
  input.push_back('\r');
  input.push_back('\n');
  auto absent = make_nmea("GPXXX,1", false);
  input.insert(input.end(), absent.begin(), absent.end());
  input.push_back('\n');
  auto invalid = make_nmea("GPGLL,4916.45,N,12311.12,W,225444,A");
  invalid.back() ^= 0x01;
  input.insert(input.end(), invalid.begin(), invalid.end());
  input.push_back('\r');

  StreamDecoder decoder;
  const auto frames = decoder.feed(input);
  require(frames.size() == 2, "NMEA valid/absent/invalid count mismatch");
  require(frames[0].checksum_status == ChecksumStatus::Valid &&
          frames[1].checksum_status == ChecksumStatus::Absent,
          "NMEA checksum states mismatch");
  require(decoder.stats().checksum_rejects == 1,
          "NMEA invalid checksum not counted");
}

void test_nmea_required_checksum_and_length_limit() {
  const auto absent = make_nmea("GPGGA,1", false);
  StreamDecoder required(65535, 66572, 64, true);
  Bytes line = absent;
  line.push_back('\n');
  require(required.feed(line).empty(), "checksum-required mode accepted absent checksum");
  require(required.stats().checksum_rejects == 1,
          "required checksum rejection not counted");

  StreamDecoder limited(32, 1029, 16, false);
  Bytes too_long{'$', 'G', 'P', 'G', 'G', 'A', ',', '1', '2', '3', '4', '5',
                 '6', '7', '8', '9', '0', '\n'};
  require(limited.feed(too_long).empty(), "overlength NMEA accepted");
  require(limited.stats().length_rejects == 1,
          "NMEA length rejection not counted");
}

void test_reset_generation_drops_partial_frame() {
  const auto frame = make_ubx({1, 2, 3, 4});
  StreamDecoder decoder;
  require(decoder.feed(frame.data(), 4).empty(), "partial UBX emitted");
  decoder.reset(9);
  require(decoder.feed(frame.data() + 4, frame.size() - 4).empty(),
          "old/new connection fragments combined");
  const auto complete = decoder.feed(frame);
  require(complete.size() == 1 && complete[0].connection_generation == 9,
          "new generation metadata mismatch");
}

void test_truncated_and_ceiling_rejection() {
  StreamDecoder decoder(32, 1029, 64);
  const Bytes short_transfer{0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x00};
  require(decoder.feed(short_transfer).empty(), "truncated UBX accepted");
  const Bytes over_ceiling{0xB5, 0x62, 0x01, 0x07, 0x21, 0x00};
  require(decoder.feed(over_ceiling).empty(), "over-ceiling UBX accepted");
  require(decoder.stats().length_rejects == 1, "UBX length reject not counted");
}

int main() {
  try {
    test_crc24q_check_value();
    test_ubx_fragmentation_and_metadata();
    test_multiple_frames_noise_and_recovery();
    test_ubx_checksum_bytes_independently();
    test_rtcm_length_boundaries();
    test_rtcm_reserved_and_crc_rejection();
    test_nmea_checksum_and_terminators();
    test_nmea_required_checksum_and_length_limit();
    test_reset_generation_drops_partial_frame();
    test_truncated_and_ceiling_rejection();
    std::cout << "10 parser test groups passed; 100 deterministic chunk segmentations passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
