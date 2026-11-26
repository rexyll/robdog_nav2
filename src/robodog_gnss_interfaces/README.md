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


## Decoded navigation epochs

The navigation observation path uses five additional messages:

- `GnssNavPvt`: decoded `gps_fix_type`, `gnss_fix_ok`, `diff_soln`, carrier solution and `invalid_llh`.
- `GnssNavHpPosLlh`: integer LLH/high-precision fields and the individual invalid-component booleans.
- `GnssNavCov`: decoded `pos_cor_valid` / `vel_cor_valid` and NED covariance upper-triangle terms in m².
- `GnssNavStatus`: decoded fix validity, differential state, `carr_soln_valid` and carrier solution for an independent quality cross-check.
- `GnssObservation`: one joined observation with GPST source timestamp/clock identity, host monotonic receive time, ENU covariance, RTK state and optional UTC/leap metadata.

All four input messages carry exact receiver iTOW, GPS week, process session ID and connection generation. They deliberately omit ROS Header timestamps: callback adapters sample host monotonic time locally. The runtime rejects stale session/generation/week data, requires an exact iTOW match across all four inputs and a valid receiver-time mapping, and does not infer receiver time from iTOW or wall clock. HPPOSLLH invalidLlh and NAV-COV posCovValid are preserved as raw validity gates; NAV-COV NED cross terms are transformed to ENU with the correct Down-axis sign changes.

These contracts describe decoded fields and do not themselves provide a UBX hardware driver or prove that a particular upstream publisher populates them correctly. Package licensing remains `UNSPECIFIED` pending repository-level license governance.


### Pinned `ublox_dgnss` source compatibility

The audited source pin `aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e` publishes the corresponding decoded ROS types on `ubx_nav_pvt`, `ubx_nav_hp_pos_llh`, `ubx_nav_cov` and `ubx_nav_status`. Its messages use `itow`, nested `gps_fix.fix_type` / `carr_soln.status`, individual HPPOSLLH invalid booleans, and `pos_cor_valid`; they do not publish the normalized process-session/connection-generation/GPS-week context. The source adapter therefore requires that context separately and ignores the upstream `Header.stamp`. The upstream node stamps that header using `rclcpp::Clock().now()` in its USB receive callback; it is a host-side arrival stamp, not receiver GPST. The bridge from this upstream package into the normalized messages must supply a trustworthy connection-generation lifecycle signal before it can be treated as reconnect-safe.


## GnssQualityStatus

`GnssQualityStatus.msg` reports the fail-closed RTK quality-gate decision
for one exact-epoch `GnssObservation`. It carries qualification/reason,
consecutive Fixed recovery count, optional uncertainty/speed metrics, and
the source session/generation/week/iTOW identity. Missing metrics use explicit
validity booleans plus zero sentinels. This status is not a NavSatFix or
motion command.
