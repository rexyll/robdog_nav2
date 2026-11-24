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
