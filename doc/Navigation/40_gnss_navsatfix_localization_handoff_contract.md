# GNSS NavSatFix / robot_localization handoff contract

日期：2026-11-26  
分支：nav

## 1. Scope and source pin

This document freezes the boundary from the project's exact-epoch
`GnssObservation` and `GnssQualityStatus` streams to
`sensor_msgs/NavSatFix` and the ROS 2 Jazzy `robot_localization`
`navsat_transform_node`.

Audited upstream sources:

- `cra-ros-pkg/robot_localization@jazzy-devel`
  - [navsat_transform_node docs](https://github.com/cra-ros-pkg/robot_localization/blob/jazzy-devel/doc/navsat_transform_node.rst)
  - [navsat_transform.cpp](https://github.com/cra-ros-pkg/robot_localization/blob/jazzy-devel/src/navsat_transform.cpp)
- [ROS 2 NavSatFix.msg](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/NavSatFix.msg)

## 2. Verified downstream contract

The Jazzy `navsat_transform_node` consumes:
- `sensor_msgs/NavSatFix` for geographic position;
- `sensor_msgs/Imu` for heading unless a supported alternative is configured;
- `nav_msgs/Odometry` for current robot pose.

It publishes `odometry/gps` in the configured world frame. Its source stores
the incoming GPS `header.stamp` as the GPS update time, uses that timestamp
for TF lookup, and stamps the GPS odometry output with that same measurement
time. The Jazzy docs require `odomN_differential=false` when fusing this GPS
odometry output into a robot_localization EKF/UKF.

The ROS `NavSatFix` contract states:
- `header.stamp` is ROS Time for the measurement; satellite time may be
  reported separately using `sensor_msgs/TimeReference`;
- `header.frame_id` is the Euclidean frame at the GNSS antenna;
- latitude/longitude/altitude use WGS-84 ellipsoid semantics;
- position covariance is ENU, row-major, in square metres.

Therefore the project's `GnssObservation.source_timestamp_ns` (GPST) cannot
be copied directly into `NavSatFix.header.stamp`. Host monotonic receive time
and callback-time `now()` are also not substitutes for the measurement
timestamp.

## 3. Timestamp policy

A future NavSatFix publisher must require all of the following:

1. An exact joined observation with matching iTOW, process session,
   connection generation and GPS week.
2. The matching quality decision is `qualified=true` and has the same
   observation identity (iTOW/session/generation/week/source clock/source
   timestamp).
3. The receiver-time mapping has `utc_stamp_valid=true`, and its UTC stamp is
   outside the mapper's leap-transition guard window.
4. The ROS clock is explicitly in real-robot SystemTime mode
   (`use_sim_time=false`), and the host SystemTime is synchronized to UTC
   within a separately measured/approved tolerance.
5. The NavSatFix stamp is constructed from the observation's exact mapped UTC
   seconds/nanoseconds, never from GPST nanoseconds or host receive time.

When `use_sim_time=true`, UTC epoch and simulation `/clock` are different
time domains. Until a separately verified UTC/GPST-to-simulation-time mapping
exists, suppress NavSatFix rather than publishing a real-world timestamp into
the simulation clock domain.

The current runtime defaults to an empty trusted leap-source allowlist. Thus
GPST observations may be valid while UTC is intentionally invalid; this must
not be bypassed to force NavSatFix publication.

## 4. Status, covariance, and frames

- `NavSatFix.status.status` is generic fix status, not an RTK carrier-state
  enum. For a qualified valid fix use `STATUS_FIX`; do not set
  `STATUS_GBAS_FIX` merely to mean RTK Fixed. RTK Fixed/Float/invalid state
  remains in `GnssQualityStatus`.
- Copy the complete 3x3 ENU covariance already generated from NAV-COV NED
  values and mark it `COVARIANCE_TYPE_KNOWN` only after the existing strict
  validity/PSD gate accepts it.
- UBX HPPOSLLH ellipsoid height maps to NavSatFix altitude. Do not silently
  substitute MSL height.
- `header.frame_id` must identify the physical GNSS antenna frame (for
  example `gnss_antenna_link`), not automatically `base_link`.
- A measured antenna-to-base_link TF is required so navsat_transform can
  compensate the antenna lever arm. Empty/missing frame IDs must not be used
  as an implicit origin assumption in the project's production configuration.

## 5. Failure propagation and recovery

The quality node publishes a separate `GnssQualityStatus`. A future publisher
must exact-join that decision to the observation and must not infer health from
NavSatFix message presence alone.

Suppress or invalidate the NavSatFix path when:
- RTK is not Fixed, the GNSS fix flag is invalid, or the quality gate is
  recovering/unqualified;
- the observation or matching decision is stale, out of order, or from an
  old session/generation/week;
- UTC/leap source is invalid or inside the transition guard;
- position covariance is invalid/degenerate;
- the ROS clock policy or antenna TF readiness is not satisfied.

A missing NavSatFix sample does not itself prove that a downstream EKF has
cleared a previously fused state. The integration must explicitly test the
selected EKF's sensor timeout, prediction-only behavior, diagnostics, and
recovery hysteresis. The quality status remains the authoritative health
signal; it must not directly command robot motion.

On recovery, require the quality gate's consecutive Fixed samples and a new
matching exact-epoch observation/decision pair. Do not replay stale cached
fixes across a receiver reconnect or clock-generation transition.

## 6. Not implemented / validation gates

This is an interface and clock-domain contract only. It does not yet add a
NavSatFix publisher, a TimeReference publisher, robot_localization launch
configuration, EKF parameters, TF broadcaster, or robot-control integration.

Next implementation slice:
1. Use the new transport-independent `GnssObservationQualityJoiner` to pair
   observations and `GnssQualityStatus` by the complete session/generation/week/
   iTOW/GPST/source-clock/monotonic identity. Its synthetic tests are in the dedicated GNSS quality CI matrix. The
   implementation head `49dc479e42d6c4141d82e907c12a7bc47cd83288` passed
   run `45812322052` on Humble/Jammy and Jazzy/Noble (208 tests in each job).
2. Add a ROS adapter that converts both ROS messages into that canonical
   identity and rejects mismatches without nearest-time pairing.
3. Add a ROS clock mode guard and explicit SystemTime/UTC mapping policy.
4. Require antenna frame and verified static extrinsic before publishing.
5. Publish NavSatFix only for qualified exact epochs with valid UTC, and test
   diagnostics/status propagation plus EKF timeout/recovery behavior.
6. Test Humble/Jazzy software integration with synthetic data, then obtain
   physical F9P/antenna/RTK/clock evidence.

Physical ZED-F9P MON-VER/CFG readback, PPS/host UTC synchronization, antenna
extrinsics, RTK correction service, actual TF timing, EKF behavior and robot
navigation remain unverified. Root repository LICENSE is absent; package
license metadata `UNSPECIFIED` is not a redistribution grant.


## 7. First clock-policy implementation slice

`robodog_gnss_time.ros_clock_policy.RosSystemTimeUtcPolicy` implements a pure, fail-closed decision gate for the timestamp preconditions above. It rejects simulation time, invalid/non-normalized UTC, leap-transition guard activation, unknown or missing host synchronization state/source, stale or future synchronization evidence, and `abs(host_utc_offset_ns) + host_utc_uncertainty_ns` above an explicitly supplied error budget. It rejects UTC seconds beyond signed 32-bit `builtin_interfaces/Time.sec` range to prevent ROS timestamp overflow.

The policy deliberately does not run `chronyc`, inspect PHC state, infer synchronization from `/clock`, or trust a static boolean. The caller must supply fresh measurements from a separate trusted host-time monitor, including the monitor's source, offset, uncertainty, and host-monotonic sample time. Both the sample and the evaluation time must use the same host monotonic clock domain. Thresholds are mandatory constructor inputs and are not production defaults. No ROS node currently supplies this evidence; until a trusted monitor and the target Orin time chain are validated, the gate remains closed and no NavSatFix is published. Synthetic policy tests do not establish actual host UTC synchronization.
