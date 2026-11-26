# Standalone UBX / RTCM3 stream decoder

This is a transport- and ROS-independent C++17 incremental parser for the parser contract in `doc/Navigation/19_ubx_rtcm_nmea_stream_parser_design_and_test_contract.md`.

## Implemented

- Incremental UBX framing (`B5 62`), declared payload length, configurable deployment payload ceiling, and CK_A/CK_B validation.
- Incremental RTCM3 framing (`D3`), reserved-bit validation, full 10-bit payload length, and CRC24Q validation.
- Arbitrary chunk boundaries, concatenated UBX/RTCM3/NMEA frames, noise skipping, checksum-failure resynchronization, connection-generation reset, frame sequence metadata, and bounded decoder storage.
- NMEA CR, LF and CRLF termination; XOR checksum validation; explicit `Valid` / `Absent` status; optional policy requiring a checksum; configurable sentence length ceiling.
- A non-blocking bounded frame queue with explicit DropOldest / RejectNewest policy, connection-generation fencing, close/drain semantics, and synchronized counters.
- A transport-neutral USB transfer lifecycle tracker and deterministic fake USB transport/session seam for submit, completion, cancellation, and shutdown-drain testing.
- No ROS 2, libusb, or external test-framework dependency.

NMEA framing, deterministic randomized segmentation tests, bounded queue tests, and the independent transfer lifecycle tracker are implemented. ROS/libusb integration and production transfer ownership/callback draining are not implemented. The fake transport validates the adapter contract only.

## Build and test

```bash
cmake -S navigation/ubx_rtcm_stream -B build/ubx_rtcm_stream -DROBODOG_ENABLE_SANITIZERS=ON
cmake --build build/ubx_rtcm_stream --parallel 2
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1 \
  ctest --test-dir build/ubx_rtcm_stream --output-on-failure
```

Optional Clang/libFuzzer smoke target (the parser library is coverage- and sanitizer-instrumented):

```bash
CC=clang CXX=clang++ cmake -S navigation/ubx_rtcm_stream -B build/ubx_rtcm_stream-fuzz -DROBODOG_BUILD_FUZZER=ON -DROBODOG_ENABLE_SANITIZERS=ON
cmake --build build/ubx_rtcm_stream-fuzz --target stream_decoder_fuzz --parallel 2
timeout 30s build/ubx_rtcm_stream-fuzz/stream_decoder_fuzz -max_total_time=20 -max_len=4096
```

ThreadSanitizer queue/concurrency checks:

```bash
cmake -S navigation/ubx_rtcm_stream -B build/ubx_rtcm_stream-tsan -DROBODOG_ENABLE_TSAN=ON
cmake --build build/ubx_rtcm_stream-tsan --parallel 2
TSAN_OPTIONS=halt_on_error=1 ctest --test-dir build/ubx_rtcm_stream-tsan --output-on-failure
```

The UBX payload ceiling is a deployment policy. The default permits the full uint16 protocol field; production configuration must set a smaller value only after the target F9P output-message whitelist and memory budget are established.

Lifecycle tracker tests are included in the same CTest suite. The tracker is a state-machine test seam only; it does not call libusb or own real transfers.
