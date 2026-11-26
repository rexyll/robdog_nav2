# robodog_gnss_time

ROS 2 Python runtime consumer for the ordered
`robodog_gnss_interfaces/msg/GnssIngressEvent` stream.

## Data flow

`/gnss/ingress` → strict ROSIDL conversion → `GnssIngressEventProcessor`
→ exact-iTOW NAV-TIMEGPS/NAV-TIMELS pairing → `GnssReceiverTime` on
`/gnss/receiver_time`.

The runtime is intentionally separate from `robodog_gnss_quality`, which
remains transport-independent and does not import `rclpy`. The node uses a
`SingleThreadedExecutor`; event ordering and mapper state are never processed
from concurrent subscription callbacks.

## Parameters

| Parameter | Default | Contract |
|---|---:|---|
| `ingress_topic` | `/gnss/ingress` | Non-empty topic |
| `mapping_topic` | `/gnss/receiver_time` | Non-empty topic |
| `qos_depth` | 256 | 1..65536; reliable, volatile, KeepLast |
| `trusted_leap_sources` | `[]` | Explicit NAV-TIMELS source IDs 1..8; empty is fail-closed |

No leap source is trusted by default. Do not add source IDs until the
receiver's firmware/protocol profile and source semantics have been audited.

## Failure behavior

- ROS ingress fields are checked as exact unsigned integer types and ranges;
  bool/float aliases are rejected.
- Process session, connection generation, ingress sequence, monotonic receive
  time, frame generation, checksum status and dropped-frame watermark are
  validated by the transport-independent processor.
- Any malformed, stale, reordered, unexplained-gap, or publishing failure
  latches the consumer faulted. The node logs fatal and shuts down; it does not
  silently resume pairing from a potentially corrupted stream.
- Receiver GPST source timestamp and UTC stamp validity are separate. An
  untrusted leap source may still produce a valid GPST mapping while
  `utc_stamp_valid=false`; the output carries explicit optional-value flags.
- `receive_monotonic_ns` is host steady-clock time, not ROS time, UTC or GPS.

## Run

Build the workspace with ROS 2 and run the publisher and consumer with
matching reliable/volatile QoS:

```bash
ros2 run robodog_gnss_transport gnss_usb_transport_node --ros-args \
  -p vendor_id:=<verified-vid> -p product_id:=<verified-pid>
ros2 run robodog_gnss_time gnss_receiver_time_node --ros-args \
  -p trusted_leap_sources:="[<audited-source-id>]"
```

The receiver source allowlist must remain empty until hardware identity and
firmware/protocol version are verified.

## Validation boundary

CI covers strict conversion, sticky fail-closed behavior, and a ROS 2
publisher/subscriber path using synthetic UBX frames on Humble/Jammy and
Jazzy/Noble. It does not validate physical USB ingress, receiver firmware,
leap transitions, DDS overload/resource limits, or downstream localization.

The package license is `UNSPECIFIED` because the repository root has no
LICENSE declaration. This is a governance blocker, not a redistribution grant.
