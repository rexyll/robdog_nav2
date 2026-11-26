# Standalone UBX / RTCM3 stream decoder

This is a transport- and ROS-independent C++17 first implementation slice for the parser contract in `doc/Navigation/19_ubx_rtcm_nmea_stream_parser_design_and_test_contract.md`.

## Implemented

- Incremental UBX framing (`B5 62`), declared payload length, configurable deployment payload ceiling, and CK_A/CK_B validation.
- Incremental RTCM3 framing (`D3`), reserved-bit validation, full 10-bit payload length, and CRC24Q validation.
- Arbitrary chunk boundaries, concatenated frames, noise skipping, checksum-failure resynchronization, connection-generation reset, frame sequence metadata, and bounded decoder storage.
- No ROS 2, libusb, or external test-framework dependency.

NMEA, property/fuzz tests, ROS/libusb integration, queue synchronization, and USB transfer lifecycle handling are not implemented in this slice.

## Build and test

```bash
cmake -S navigation/ubx_rtcm_stream -B build/ubx_rtcm_stream -DROBODOG_ENABLE_SANITIZERS=ON
cmake --build build/ubx_rtcm_stream --parallel 2
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1 \
  ctest --test-dir build/ubx_rtcm_stream --output-on-failure
```

The UBX payload ceiling is a deployment policy. The default permits the full uint16 protocol field; production configuration must set a smaller value only after the target F9P output-message whitelist and memory budget are established.
