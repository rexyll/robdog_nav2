# GNSS RTK quality gate ROS 2 runtime contract

日期：2026-11-26  
分支：nav

## 1. Data flow

```text
GnssUsbTransportNode
  -> /gnss/ingress (ordered, session/generation fenced)
  -> GnssIngressConsumer + UbxReceiverTimeMapper
  -> UbxRawNavEpochAssembler
  -> /gnss/observation (exact iTOW + valid GPST mapping)
  -> GnssRtkQualityNode
  -> /gnss/quality (GnssQualityStatus)
```

The quality algorithm remains transport-independent in
`robodog_gnss_quality.quality_gate.RtkQualityGate`. The ROS runtime lives in
`robodog_gnss_time` and does not change the pure gate's input contract.

## 2. Input contract and clock domains

The node consumes `GnssObservation`, which must already represent one exact
NAV-PVT/HPPOSLLH/COV/STATUS iTOW joined with a valid GPST source-time mapping.
It does not use ROS Header.stamp or wall clock. The gate compares
`receive_monotonic_ns` with Python `time.monotonic_ns()` for freshness and
compares `source_timestamp_ns` only within the same `source_clock_id`.

The runtime requires the canonical clock ID:

```text
{process_session_id}:connection:{connection_generation}:gps-week:{gps_week}
```

A forward context transition resets gate hysteresis and position-jump
reference. Within the same process session, a lower connection generation or
lower GPS week is rejected as OUT_OF_ORDER. Receive monotonic timestamps must
strictly increase; delayed/duplicate observations are reported as
OUT_OF_ORDER and do not update gate state.

## 3. Output contract

`GnssQualityStatus` contains:
- `qualified` and stable reason string from `RejectionReason`;
- consecutive Fixed sample count required for recovery hysteresis;
- horizontal/vertical standard deviation and estimated speed, each with
  explicit validity boolean and zero sentinel;
- iTOW, GPS week, process session, connection generation, source GPST time,
  source clock ID, and host monotonic receive timestamp.

The node publishes an initial `no_data` state. A host-monotonic watchdog
re-evaluates the last observation after `max_fix_age_ns`; stale input produces
`fix_stale` and withdraws qualification. It publishes a new status only
when the decision signature changes, avoiding repeated stale messages.

## 4. Configuration

| Parameter | Default | Meaning |
|---|---:|---|
| observation_topic | /gnss/observation | Exact-epoch raw USB observation |
| status_topic | /gnss/quality | Quality decision output |
| qos_depth | 64 | Reliable/volatile KeepLast |
| max_fix_age_ns | 200000000 | Host monotonic freshness bound |
| max_horizontal_std_m | 0.25 | Horizontal uncertainty limit |
| max_vertical_std_m | 0.5 | Vertical uncertainty limit |
| max_plausible_speed_mps | 5.0 | 3D jump plausibility limit |
| required_consecutive_fixed_samples | 3 | Recovery hysteresis |
| max_jump_reference_interval_ns | 2000000000 | Continuity window |
| allow_approximated_covariance | false | Covariance policy |

These defaults are development/test placeholders only. They are not measured
ZED-F9P accuracy or validated operational limits. Production configuration
requires actual receiver/antenna/RTK measurements, intended robot dynamics,
integrity analysis and system-level safety approval.

## 5. Failure and safety boundaries

- Invalid ROS field types/ranges or noncanonical source clock identity latch
  the node faulted and shut down.
- Old generation/week and non-increasing receive timestamps produce
  OUT_OF_ORDER and do not mutate active gate state.
- A forward source context resets recovery hysteresis.
- No fresh observation means no qualified state: watchdog emits FIX_STALE.
- A quality decision is not a NavSatFix, TF transform, EKF measurement,
  safety-rated output, or motion authorization.
- No localization consumer is connected yet. UTC validity remains separate
  from GPST; NavSatFix timestamp conversion needs a separately approved
  ROS-time policy and must not silently treat GPST as UTC.

## 6. Validation status

The dedicated `gnss-time-node-ci.yml` workflow builds/tests the interface and
runtime package on ROS 2 Humble/Jammy and Jazzy/Noble. Integration tests cover
recovery hysteresis, generation rollback rejection and stale watchdog
withdrawal using synthetic messages. CI results are pending for this first
implementation.

Physical ZED-F9P USB, MON-VER/firmware and CFG readback, NAV-COV support,
RTK correction service, output rate/jitter, DDS resource exhaustion, and
localization/EKF integration remain unverified. Root repository LICENSE is
absent; ROS package metadata remains `UNSPECIFIED`, not a redistribution
grant.
