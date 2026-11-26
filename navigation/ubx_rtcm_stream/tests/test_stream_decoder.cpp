#include "robodog_protocol/stream_decoder.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using robodog_protocol::Frame;
using robodog_protocol::Protocol;
using robodog_protocol::StreamDecoder;
using Bytes = std::vector<std::uint8_t>;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

Bytes make_ubx(Bytes payload, std::uint8_t cls = 0x01, std::uint8_t id = 0x07) {
  const auto n = static_cast<std::uint16_t>(payload.size());
  Bytes out{0xB5, 0x62, cls, id,
            static_cast<std::uint8_t>(n & 0xFF),
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
  Bytes out{0xD3,
            static_cast<std::uint8_t>((payload_size >> 8) & 0x03),
            static_cast<std::uint8_t>(payload_size & 0xFF)};
  for (std::size_t i = 0; i < payload_size; ++i)
    out.push_back(static_cast<std::uint8_t>((i * 37U + 11U) & 0xFFU));
  const auto crc = StreamDecoder::crc24q(out.data(), out.size());
  out.push_back(static_cast<std::uint8_t>((crc >> 16) & 0xFF));
  out.push_back(static_cast<std::uint8_t>((crc >> 8) & 0xFF));
  out.push_back(static_cast<std::uint8_t>(crc & 0xFF));
  return out;
}

void test_crc24q_check_value() {
  const std::string s = "123456789";
  const auto crc = StreamDecoder::crc24q(
      reinterpret_cast<const std::uint8_t*>(s.data()), s.size());
  require(crc == 0xCDE703U, "CRC24Q check value mismatch");
}

void test_ubx_fragmentation_and_metadata() {
  const auto frame = make_ubx({0x10, 0x20, 0x30});
  for (std::size_t split = 0; split <= frame.size(); ++split) {
    StreamDecoder decoder;
    auto a = decoder.feed(frame.data(), split);
    auto b = decoder.feed(frame.data() + split, frame.size() - split);
    require((split == frame.size()) ? a.size() == 1 : a.empty(),
            "UBX emitted before complete frame");
    require((split == frame.size() ? b.empty() : b.size() == 1) &&
                (split == frame.size() ? a[0].protocol : b[0].protocol) == Protocol::Ubx,
            "UBX split frame not decoded");
    const auto& emitted = split == frame.size() ? a[0] : b[0];
    require(emitted.bytes == frame && emitted.sequence == 0 &&
                emitted.connection_generation == 0,
            "UBX frame metadata mismatch");
  }
}

void test_multiple_frames_noise_and_recovery() {
  auto u = make_ubx({1, 2, 3});
  auto r = make_rtcm(256);
  Bytes stream{0x00, 0x44, 0x99};
  stream.insert(stream.end(), u.begin(), u.end());
  stream.push_back(0x13);
  stream.insert(stream.end(), r.begin(), r.end());
  auto bad = make_ubx({4, 5});
  bad[bad.size() - 2] ^= 0x01;
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.insert(stream.end(), u.begin(), u.end());

  StreamDecoder decoder;
  std::vector<Frame> frames;
  for (auto byte : stream) {
    auto part = decoder.feed(&byte, 1);
    frames.insert(frames.end(), part.begin(), part.end());
  }
  require(frames.size() == 3, "multi-frame/noise/recovery count mismatch");
  require(frames[0].protocol == Protocol::Ubx &&
              frames[1].protocol == Protocol::Rtcm3 &&
              frames[2].protocol == Protocol::Ubx,
          "mixed protocol order mismatch");
  require(frames[0].sequence == 0 && frames[2].sequence == 2,
          "monotonic sequence mismatch");
  require(decoder.stats().checksum_rejects >= 1, "bad UBX checksum not counted");
}

void test_ubx_checksum_bytes_independently() {
  const auto good = make_ubx({0xAA, 0x55});
  for (const auto index : {good.size() - 2, good.size() - 1}) {
    auto bad = good;
    bad[index] ^= 0x80;
    StreamDecoder decoder;
    auto frames = decoder.feed(bad);
    require(frames.empty(), "corrupt UBX checksum accepted");
    require(decoder.stats().checksum_rejects == 1,
            "UBX checksum rejection not counted");
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
                frames[0].bytes == frame,
            "RTCM length boundary failed: " + std::to_string(length));
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
  auto frames = decoder.feed(input);
  require(frames.size() == 1 && frames[0].bytes == valid,
          "RTCM malformed frame recovery failed");
  require(decoder.stats().malformed_headers >= 1,
          "RTCM reserved bits not counted");
  require(decoder.stats().checksum_rejects >= 1,
          "RTCM CRC failure not counted");
}

void test_reset_generation_drops_partial_frame() {
  const auto frame = make_ubx({1, 2, 3, 4});
  StreamDecoder decoder;
  auto first = decoder.feed(frame.data(), 4);
  require(first.empty(), "partial UBX unexpectedly emitted");
  decoder.reset(9);
  auto rest = decoder.feed(frame.data() + 4, frame.size() - 4);
  require(rest.empty(), "old/new connection fragments were combined");
  auto complete = decoder.feed(frame);
  require(complete.size() == 1 && complete[0].connection_generation == 9,
          "new generation metadata mismatch");
}

void test_truncated_and_ceiling_rejection() {
  StreamDecoder decoder(32, 1029);
  const Bytes short_transfer{0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x00};
  require(decoder.feed(short_transfer).empty(), "truncated UBX accepted");
  const Bytes over_ceiling{0xB5, 0x62, 0x01, 0x07, 0x21, 0x00};
  require(decoder.feed(over_ceiling).empty(), "over-ceiling UBX accepted");
  require(decoder.stats().length_rejects == 1, "length reject not counted");
}

int main() {
  try {
    test_crc24q_check_value();
    test_ubx_fragmentation_and_metadata();
    test_multiple_frames_noise_and_recovery();
    test_ubx_checksum_bytes_independently();
    test_rtcm_length_boundaries();
    test_rtcm_reserved_and_crc_rejection();
    test_reset_generation_drops_partial_frame();
    test_truncated_and_ceiling_rejection();
    std::cout << "8 parser test groups passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
