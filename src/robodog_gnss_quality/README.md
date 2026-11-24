# robodog_gnss_quality

Transport-independent, fail-closed RTK quality gate core for the RobDog Nav2 project.

## Scope

This package implements the deterministic acceptance policy that must run before a GNSS position is allowed into the global localization chain. It is deliberately a pure Python package in v0.1.0: there is no ROS node, receiver socket, UBX decoder, NavSatFix publisher, or robot-control interface yet.

## Inputs

Each GnssObservation represents one already epoch-aligned measurement and includes:

- receiver source timestamp and explicit source-clock epoch ID (receiver reboot/timebase change resets continuity);
- host monotonic receive timestamp (freshness only);
- NavSatFix latitude, longitude, altitude, status and 3x3 ENU position covariance;
- covariance type;
- separate receiver GNSS-fix-valid flag;
- explicit carrier-phase solution: NO_SOLUTION, FLOAT, FIXED, or UNKNOWN.

The adapter must not infer RTK Fixed from NavSatStatus alone. ROS NavSatStatus reports generic fix/SBAS/GBAS status, while u-blox NAV-PVT separately reports GNSS fix validity and carrier-phase solution. The NAV-PVT and NavSatFix records must be paired on the same GNSS epoch (e.g. receiver iTOW where supported) before constructing the observation. A mismatched epoch must be rejected upstream.

## Acceptance policy

1. Require a valid GNSS fix and explicit RTK Fixed carrier solution.
2. Require a fresh host receive timestamp and monotonic source epochs.
3. Validate coordinate ranges and finite values.
4. Require known covariance (diagonal-known or full-known by default); approximated covariance is disabled unless explicitly enabled.
5. Validate covariance shape, finiteness, symmetry and positive semidefiniteness; reject zero/degenerate covariance rather than interpreting it as perfect accuracy. DIAGONAL_KNOWN requires zero off-diagonal entries.
6. Enforce configured maximum horizontal and vertical standard deviation.
7. Reject physically implausible 3D position jumps using a haversine horizontal distance plus altitude delta.
8. Require a configurable number of consecutive acceptable Fixed epochs before declaring the stream qualified. A Float/invalid/stale sample resets recovery.
9. After a source-time gap longer than the configured continuity window, restart qualification rather than bridging an unobserved interval.

No numerical accuracy, age, speed, or sensor rate is claimed for the selected ZED-F9P. Thresholds are mandatory configuration and must be chosen from measured receiver behavior and the intended operation envelope.

## Build and test

    source /opt/ros/humble/setup.bash  # or jazzy
    cd <workspace-root>
    colcon build --packages-select robodog_gnss_quality
    colcon test --packages-select robodog_gnss_quality
    colcon test-result --verbose

The core tests use only Python's standard library and pytest/unittest. This package does not depend on ROS messages so the algorithm can be tested on both Humble and Jazzy before the receiver-specific ROS adapter is implemented.

## Safety and integration boundary

- Not yet wired to navsat_transform_node, robot_localization, or an EKF.
- Does not output NavSatFix or Odometry; it outputs only a decision object in-process.
- Does not publish TF or map -> odom.
- RTK Float, single, differential, unknown or stale solutions are never accepted as RTK Fixed.
- A mathematically PSD covariance is not proof that the receiver uncertainty is statistically calibrated. The adapter/source quality review remains required.
- The gate is not a safety-rated system and does not authorize autonomous movement.


## UBX-NAV-PVT quality-field adapter contract

The package now includes a receiver-message helper for the UBX-NAV-PVT flags byte. It decodes bit 0 as GNSSFixOK and bits 7:6 as carrSoln (0 no carrier solution, 1 Float, 2 Fixed, 3 reserved/unknown). It does not parse serial bytes or subscribe to a specific ROS driver.

The helper pairs a position record and NAV-PVT quality record only when their receiver iTOW values match exactly and their host monotonic receive times are within an explicitly supplied skew limit. It rejects mismatched epochs; it never pairs by "nearest arrival" or ROS header timestamps alone.

The position record must preserve a receiver source timestamp and a source_clock_id identifying the GPS week/receiver clock epoch. iTOW is a week-relative counter and must not be treated as a globally monotonic timestamp. The chosen ROS driver must be checked to confirm it exposes the required UBX fields and a reliable way to associate the matching NavSatFix epoch. That driver selection and full ROS adapter remain open.


## Raw u-blox UBX exact-epoch assembly

For the pinned `aussierobots/ublox_dgnss` driver, the preferred input to the quality gate is the original UBX-derived topic set, not the driver's convenience `NavSatFix` conversion node. The pure helper `ubx_nav_pvt.build_observation_from_ubx_epochs()` accepts decoded fields from:

- `UBXNavPVT`: `iTOW`, `gnss_fix_ok`, and carrier-solution state;
- `UBXNavHPPosLLH`: `iTOW`, high-precision LLH components and invalid-component flags;
- `UBXNavCov`: `iTOW`, `pos_cor_valid`, and the six upper-triangle NED position-covariance elements;
- `UBXNavStatus`: `iTOW`, GNSS validity, carrier-solution validity and carrier-solution state.

All four records must have exactly equal `iTOW`. The helper also requires their host monotonic receive timestamps to fit within the configured maximum cross-topic skew. A nearest-arrival match is never attempted. NAV-PVT and NAV-STATUS quality fields must agree, HPPOSLLH invalid flags must all be clear, and NAV-COV must explicitly mark its position covariance valid.

### Covariance frame conversion

UBX-NAV-COV expresses position covariance in NED. The helper converts the six values to a symmetric row-major ENU matrix with

```text
C_NED = [[NN, NE, ND],
         [NE, EE, ED],
         [ND, ED, DD]]

R_NED_to_ENU = [[0, 1,  0],
                [1, 0,  0],
                [0, 0, -1]]

C_ENU = R_NED_to_ENU * C_NED * transpose(R_NED_to_ENU)
```

The resulting row-major values are `[EE, NE, -ED, NE, NN, -ND, -ED, -ND, DD]`. Cross terms involving Down change sign. The existing RTK gate subsequently validates finite values, covariance symmetry/PSD, non-degeneracy and uncertainty limits.

### Time-domain boundary

The caller must supply `source_timestamp_ns` from a separately validated receiver-time mapper and a `source_clock_id` that identifies the GPS week and receiver clock generation. The helper deliberately does not synthesize a globally monotonic timestamp from iTOW: iTOW is week-relative and wraps every GPS week.

The pinned driver's ROS `header.stamp` is populated from the host USB receive path; it must not be treated as receiver measurement time or host steady-clock freshness. A future ROS adapter must timestamp each callback using one host monotonic clock, maintain bounded per-topic epoch caches, and clear them on receiver clock-generation changes. `NAV-STATUS.msss` can help detect a receiver reset, but its 32-bit startup counter also wraps and must not be used alone as a permanent generation identifier.

This helper is pure algorithm code. It is not a serial/USB parser, ROS subscription node, receiver-time mapper, NavSatFix publisher, PPS driver, NTRIP client, or EKF integration.
