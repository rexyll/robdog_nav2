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
| `observation_topic` | `/gnss/observation_decoded` | Joined observation from normalized topics |
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


## Preferred raw-USB observation path

The preferred end-to-end path is now integrated into `gnss_receiver_time_node`:

```text
/gnss/ingress
  ├─ ordered GenerationStarted / DeviceLost / TransportFault / Stopped
  ├─ UBX-NAV-TIMEGPS + UBX-NAV-TIMELS
  │    └─ validated receiver-time mapping
  └─ UBX-NAV-PVT / HPPOSLLH / COV / STATUS raw frames
       └─ exact iTOW + same process session / connection generation
            + matching validated GPS week/time mapping
              -> /gnss/observation
```

The node uses `UbxIngressEventProcessor` for ordered lifecycle/time mapping and
`UbxRawNavEpochAssembler` for raw NAV frame decoding. The raw decoder validates
UBX envelope/checksum again, requires exact payload lengths and supported
message versions, and extracts:
- NAV-PVT class/id `01/07`, 92-byte payload;
- NAV-HPPOSLLH `01/14`, 36-byte payload with all eight individual invalid bits;
- NAV-COV `01/36`, 64-byte payload;
- NAV-STATUS `01/03`, 16-byte payload.

The transport event supplies process session ID, connection generation,
ingress sequence and host monotonic receive timestamp. GPS week is not guessed
from iTOW: raw NAV records are held in a bounded pre-mapping cache until a
valid TIMEGPS/TIMELS mapping with the same iTOW and the same session/generation
supplies the week. The four decoded epochs then pass through the same
`UbxRosEpochAdapter` exact join and covariance/quality construction. The
assembler enforces callback receive skew, cache capacity, expiry, duplicate
conflict retirement and generation reset.

This raw-ingress path is preferred over the optional `gnss_observation_node`
decoded-topic path because it has authoritative transport generation metadata.
The decoded-topic node remains useful when an external driver supplies the
normalized `GnssNav*` messages; it publishes to `/gnss/observation_decoded` by
default so it cannot silently compete with the preferred raw-ingress
`/gnss/observation` publisher. The pinned `ublox_dgnss` ROSIDL does not carry
connection generation and therefore needs an independently trusted lifecycle
bridge before reconnect safety can be claimed.

### Raw-path parameters

`gnss_receiver_time_node` adds:
- `observation_topic` (default `/gnss/observation`);
- `max_pending_epochs` (default 64, range 1..4096);
- `max_pair_receive_skew_ns` (default 250 ms);
- `max_pending_age_ns` (default 1 s).

The four-stream raw path is synthetic-CI validated; physical ZED-F9P message
availability, NAV-COV firmware support, output rates, reconnect behavior and
USB/DDS overload handling remain unverified on target hardware.


## RTK quality gate ROS runtime

The `gnss_rtk_quality_node` executable subscribes to the preferred raw-path
`/gnss/observation` output and publishes one `GnssQualityStatus` decision
per observation on `/gnss/quality`. The pure `RtkQualityGate` remains in
`robodog_gnss_quality`; this ROS adapter does not add ROS dependencies to the
algorithm package.

The node uses Reliable/Volatile/KeepLast QoS and a SingleThreadedExecutor.
It checks the canonical `source_clock_id` against
`process_session_id/connection_generation/gps_week`, rejects receive-time
regression and stale generation/week rollback, resets recovery hysteresis on
a forward clock-context transition, and has a host-monotonic watchdog that
withdraws qualification when observations stop arriving. The output is a
decision/status interface, not a NavSatFix, TF, localization measurement, or
motion authorization.

Run:

```bash
ros2 run robodog_gnss_time gnss_rtk_quality_node --ros-args \\
  -p max_fix_age_ns:=200000000 \\
  -p max_horizontal_std_m:=0.25 \\
  -p max_vertical_std_m:=0.5 \\
  -p max_plausible_speed_mps:=5.0 \\
  -p required_consecutive_fixed_samples:=3
```

The current parameter defaults are development/test placeholders, not
ZED-F9P accuracy claims or deployment thresholds. Select production values
only from measured receiver output, expected dynamics, antenna installation,
and the system-level integrity/risk analysis. The node's `qualified` result
must not directly command robot motion.

Dedicated Humble/Jazzy CI is required for the ROS message generation, runtime
node, recovery hysteresis, generation rollback, and stale watchdog. No physical
receiver, RTK correction link, DDS overload, or localization consumer has been
validated by these synthetic tests.



## Exact observation / quality ROS adapter

`RosObservationQualityJoinAdapter` converts `GnssObservation` and
`GnssQualityStatus` messages into the transport-independent canonical
identity and `QualityDecision`, then delegates pairing to
`GnssObservationQualityJoiner`.

The identity includes process session, connection generation, GPS week,
iTOW, GPST source timestamp, canonical source clock ID and host monotonic
receive timestamp. No nearest-time or approximate pairing is allowed.
Observation and decision may arrive in either order; only an exact matching
`qualified=true` / `accepted` pair is emitted. Unqualified decisions retire
the matching identity. Pending entries and expiry remain bounded by the core
joiner.

The adapter rejects unknown reason strings, inconsistent qualified/reason
pairs, bool-as-integer aliases, invalid optional metric sentinels, non-finite
metrics, and source-clock IDs inconsistent with session/generation/week.
The new adapter tests run under the dedicated Humble/Jazzy GNSS runtime CI.

This slice only creates the strict message-to-core adapter and its tests. It
does not yet publish NavSatFix. ROS clock policy, host UTC synchronization,
antenna static-extrinsic readiness, NavSatFix publication and EKF timeout /
recovery integration remain separate gates.


### Per-epoch quality publication and startup sentinel

The quality node publishes an identity-bearing decision for **every** observation, including consecutive samples whose quality state and metrics remain unchanged. Signature deduplication is limited to repeated watchdog states; otherwise a stable accepted state could starve the downstream exact identity joiner. The one startup `no_data` status has no epoch identity and is ignored only when every identity field and optional metric is at its exact default sentinel. A malformed or partially populated `no_data` message is rejected rather than silently ignored.

Out-of-order observations also receive one identity-bearing rejection per input epoch; only repeated watchdog timer emissions are deduplicated. This preserves the exact message-pair contract for rejected samples as well as accepted/recovering samples.


## ROS SystemTime / UTC policy gate

`ros_clock_policy.py` adds a transport-independent, fail-closed policy check for a future NavSatFix publisher. It rejects `use_sim_time=true`, invalid or non-normalized mapped UTC, active leap-transition guard, missing/unknown host synchronization state, missing synchronization source, stale/future host-sync evidence, and host UTC offset plus uncertainty outside an explicitly configured timing budget. It also rejects UTC seconds outside the signed 32-bit `builtin_interfaces/Time.sec` range instead of allowing timestamp overflow.

The gate does **not** discover or prove host clock synchronization. A separate trusted runtime monitor must provide a fresh synchronization state, source, signed offset, uncertainty, and host-monotonic sample time. The source identifier must also appear in an explicit immutable allowlist configured from the deployed host-time architecture. Until that monitor exists and is validated against the robot's PHC/chrony/PTP configuration, callers must provide no evidence and the policy remains closed. Thresholds have no production defaults; they must come from the system timing/integrity budget. The policy gate is not yet connected to a NavSatFix publisher or robot_localization.


### HostTimeSyncStatus adapter

`HostTimeSyncStatusAdapter` validates the host clock evidence message against the active Linux boot ID and rejects malformed state/source/offset/uncertainty, cross-boot samples, repeated or reordered sequence numbers, and non-increasing CLOCK_MONOTONIC sample timestamps. It preserves `unknown`, `unsynchronized` and `fault` states as non-qualified evidence for the clock policy to reject. It does not launch chrony/ptp4l/pmc, read PHC state, or create the host boot ID; a trusted monitor must provide those inputs. The adapter and interface tests are synthetic only.
