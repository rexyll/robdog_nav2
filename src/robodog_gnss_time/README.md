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


## Four-stream receiver-time observation join

A second executable, `gnss_observation_node`, subscribes to decoded receiver
navigation topics and the independently validated `GnssReceiverTime` stream:

```text
/gnss/nav_pvt -----------\
/gnss/nav_hp_pos_llh -----+--> exact iTOW + session/generation/week join
/gnss/nav_cov -----------+        + valid receiver-time mapping
/gnss/nav_status --------/        -> /gnss/observation
/gnss/receiver_time -----/
```

The input wire contracts are `GnssNavPvt`, `GnssNavHpPosLlh`,
`GnssNavCov`, and `GnssNavStatus`. They carry the decoded semantic UBX fields
plus process session, connection generation and GPS week. The pinned
`ublox_dgnss` ROSIDL exposes decoded flags (not the original full flag bytes),
so these adapters do not claim raw-byte preservation. They intentionally omit
ROS Header stamps; the node samples `time.monotonic_ns()` at callback receipt
for transport freshness and skew checks.

A valid receiver-time mapping is the only authority that activates a changed
session/generation/week. Before the first mapping, raw navigation records may
be held in the bounded epoch cache but cannot produce an observation. Raw
records from a different generation are dropped until a matching valid mapping
activates that generation. The join requires exact iTOW equality across all
four streams and a valid mapping for that same iTOW. It cross-checks PVT and
STATUS fix/carrier state, requires HPPOSLLH LLH validity and NAV-COV
position-covariance validity, and converts the NAV-COV NED covariance into
row-major ENU with the correct sign changes for Down cross-terms.

The output `GnssObservation` carries GPST source time and clock identity,
host monotonic receive time, position, RTK state, ENU covariance, and optional
UTC/leap metadata. GPST validity does not imply UTC validity. This node only
assembles and publishes the observation; the separate RTK quality gate and
localization consumer remain subsequent stages.

### Parameters

| Parameter | Default | Contract |
|---|---:|---|
| `nav_pvt_topic` | `/gnss/nav_pvt` | Decoded NAV-PVT stream |
| `nav_hp_pos_llh_topic` | `/gnss/nav_hp_pos_llh` | Decoded HPPOSLLH stream |
| `nav_cov_topic` | `/gnss/nav_cov` | Decoded NAV-COV stream |
| `nav_status_topic` | `/gnss/nav_status` | Decoded NAV-STATUS stream |
| `receiver_time_topic` | `/gnss/receiver_time` | Validated receiver-time mapping |
| `observation_topic` | `/gnss/observation` | Joined observation |
| `qos_depth` | 256 | Reliable, volatile, KeepLast depth |
| `max_pending_epochs` | 64 | Bounded iTOW cache, range 1..4096 |
| `max_pair_receive_skew_ns` | 250000000 | Maximum 250 ms callback receive skew |
| `max_pending_age_ns` | 1000000000 | Maximum 1 s incomplete-epoch age |

The decoded-topic publishers must populate the documented semantic fields and
session/generation/week fields from the same receiver connection. A tested
field-level adapter now maps the pinned upstream `ublox_ubx_msgs` message shapes
into the normalized records; however, the upstream ROS messages themselves do
not carry session/generation/week. A reconnect-safe bridge still requires a
trusted lifecycle/generation signal. No concrete upstream driver has yet been
certified to publish these normalized custom contracts.
The new CI validates synthetic ROS messages and the software join path only;
it does not establish real driver compatibility, physical ZED-F9P behavior,
or localization performance.
