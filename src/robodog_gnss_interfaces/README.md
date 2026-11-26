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

The package is an interface-only slice. It does not implement the publisher,
USB device selection, rclcpp component, DDS QoS, subscriber executor, or
callback-drain barrier. Those remain separate implementation and validation
gates. Package license metadata is currently `UNSPECIFIED` because the
repository has no declared root license; public redistribution is not approved
by this placeholder.
