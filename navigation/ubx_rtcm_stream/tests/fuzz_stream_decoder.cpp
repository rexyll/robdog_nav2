#include "robodog_protocol/stream_decoder.hpp"

#include <cstddef>
#include <cstdint>

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  robodog_protocol::StreamDecoder decoder(4096, 8192, 1024, false);
  std::size_t offset = 0;
  while (offset < size) {
    const std::size_t chunk = 1 + (data[offset] % 31U);
    const std::size_t count = chunk < size - offset ? chunk : size - offset;
    (void)decoder.feed(data + offset, count);
    offset += count;
  }
  return 0;
}
