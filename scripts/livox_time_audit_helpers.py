"""Pure timestamp audit helpers; no ROS runtime required."""
from __future__ import annotations

import bisect
import math
from typing import Any, Sequence

NSEC_PER_SEC = 1_000_000_000


def validate_cli_thresholds(
    max_header_timebase_delta_ns: int,
    max_nearest_lidar_imu_p95_ms: float | None,
) -> None:
    """Reject invalid CLI limits before opening or scanning a bag."""
    if max_header_timebase_delta_ns < 0:
        raise ValueError("max-header-timebase-delta-ns must be non-negative")
    if max_nearest_lidar_imu_p95_ms is not None:
        if (
            not math.isfinite(max_nearest_lidar_imu_p95_ms)
            or max_nearest_lidar_imu_p95_ms < 0
        ):
            raise ValueError(
                "max-nearest-lidar-imu-p95-ms must be a finite, non-negative number"
            )


def percentile(values: Sequence[float], p: float) -> float | None:
    if not 0.0 <= p <= 100.0:
        raise ValueError("percentile must be within [0, 100]")
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * p / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction


def stamp_to_ns(stamp: Any) -> int:
    sec = int(stamp.sec)
    nanosec = int(stamp.nanosec)
    if sec < 0:
        raise ValueError("ROS stamp seconds must be non-negative")
    if not 0 <= nanosec < NSEC_PER_SEC:
        raise ValueError(f"nanosec outside [0, 1e9): {nanosec}")
    return sec * NSEC_PER_SEC + nanosec


def summarize_values(values: Sequence[int | float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "min": None, "p50": None, "p95": None, "p99": None, "max": None}
    converted = [float(value) for value in values]
    return {
        "count": len(converted),
        "min": min(converted),
        "p50": percentile(converted, 50),
        "p95": percentile(converted, 95),
        "p99": percentile(converted, 99),
        "max": max(converted),
    }


def summarize_time_series(times_ns: Sequence[int]) -> dict[str, Any]:
    periods = [right - left for left, right in zip(times_ns, times_ns[1:])]
    positive_periods = [period for period in periods if period > 0]
    duplicate_count = sum(period == 0 for period in periods)
    backward_count = sum(period < 0 for period in periods)
    period_summary = summarize_values(positive_periods)
    median_period = period_summary["p50"]
    frequency = (
        NSEC_PER_SEC / median_period
        if median_period is not None and median_period > 0
        else None
    )
    return {
        "message_count": len(times_ns),
        "duplicate_timestamp_count": duplicate_count,
        "backward_timestamp_count": backward_count,
        "positive_period_ns": period_summary,
        "median_frequency_hz": frequency,
    }


def nearest_neighbor_deltas_ns(
    reference_times_ns: Sequence[int],
    sample_times_ns: Sequence[int],
) -> list[int]:
    if not reference_times_ns or not sample_times_ns:
        return []
    ordered_samples = sorted(sample_times_ns)
    deltas: list[int] = []
    for reference in reference_times_ns:
        index = bisect.bisect_left(ordered_samples, reference)
        candidates = []
        if index < len(ordered_samples):
            candidates.append(ordered_samples[index])
        if index > 0:
            candidates.append(ordered_samples[index - 1])
        deltas.append(min(abs(reference - candidate) for candidate in candidates))
    return deltas



def summarize_timestamp_pairing(
    sensor_times_ns: Sequence[int],
    receive_times_ns: Sequence[int],
) -> dict[str, Any]:
    """Describe paired bag receive minus sensor timestamps without assuming a shared clock."""
    if len(sensor_times_ns) != len(receive_times_ns):
        raise ValueError("sensor and receive timestamp counts must match")
    deltas = [
        int(receive) - int(sensor)
        for sensor, receive in zip(sensor_times_ns, receive_times_ns)
    ]
    return {
        "pair_count": len(deltas),
        "receive_minus_sensor_ns": summarize_values(deltas),
        "same_clock_domain_proven": False,
        "interpretation": "descriptive_only_until_clock_domains_and_mapping_are_verified",
    }
