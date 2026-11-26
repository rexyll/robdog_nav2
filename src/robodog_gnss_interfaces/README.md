# robodog_gnss_interfaces

ROS 2 wire contract for the ordered GNSS USB ingress stream.

## GnssIngressEvent

One topic message represents exactly one event from the C++ `GnssIngressQueue`:
`GenerationStarted`, `Frame`, `DeviceLost`, `TransportFault`, or `Stopped`.

The message deliberately does not include `std_msgs/Header`: `receive_monotonic_ns`
is captured from the host steady clock and must not be interpreted as ROS time,
UTC, GPS time, or a publish timestamp.

For frame events, `protocol`, `checksum_status`, `frame_sequence`,
`frame_connection_generation`, and `frame_bytes` preserve the decoded C++
Frame metadata. Consumers must verify
`frame_connection_generation == connection_generation`, validate the protocol
and checksum status, and process events in strictly increasing
`ingress_sequence` order. `dropped_frame_events_total` is a per-event prefix
watermark: it counts only evictions whose dropped frame sequence precedes the
event. A later eviction must never retroactively alter an earlier lifecycle
event. Lifecycle events carry no frame bytes and use `PROTOCOL_NONE`.

## GnssReceiverTime

`GnssReceiverTime` is emitted only after the Python raw-ingress processor pairs
NAV-TIMEGPS and NAV-TIMELS with the same iTOW and active process/connection
generation. `source_timestamp_ns` is receiver GPST in `source_clock_id`;
`receive_monotonic_ns` remains host steady-clock arrival time.

UTC values and optional receiver metadata use explicit `*_valid` flags. In
particular, `valid=true` can coexist with `utc_stamp_valid=false`: GPST may be
valid while the leap source is not trusted. Consumers must not synthesize UTC
from GPST unless `utc_stamp_valid` is true. A missing event countdown is
represented by `time_to_leap_event_seconds_valid=false`, not a fabricated
zero.

The package remains an interface-only slice. The C++ publisher/runtime and
Python consumer/runtime live in separate packages and have independent CI
gates. The Python consumer uses a single-threaded executor and latches any
malformed/reordered ingress or mapping-publish failure as a process-level
fail-closed condition.

Package license metadata is currently `UNSPECIFIED` because the repository
has no declared root license; public redistribution is not approved by this
placeholder.
