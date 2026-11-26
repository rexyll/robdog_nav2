"""Strict conversion of a completed exact-epoch result to ROSIDL."""

from robodog_gnss_quality.quality_gate import CarrierSolution, CovarianceType
from robodog_gnss_quality.ros_epoch_adapter import UbxRosEpochResult


_CARRIER_TO_CODE = {
    CarrierSolution.NO_SOLUTION: 0,
    CarrierSolution.FLOAT: 1,
    CarrierSolution.FIXED: 2,
    CarrierSolution.UNKNOWN: 3,
}


def populate_gnss_observation_message(result, message):
    if not isinstance(result, UbxRosEpochResult):
        raise ValueError("result must be an UbxRosEpochResult")
    observation = result.observation
    mapping = result.time_mapping

    message.i_tow_ms = mapping.i_tow_ms
    message.gps_week = mapping.gps_week
    message.process_session_id = mapping.process_session_id
    message.connection_generation = mapping.connection_generation
    message.source_timestamp_ns = observation.source_timestamp_ns
    message.source_clock_id = observation.source_clock_id
    message.receive_monotonic_ns = observation.receive_monotonic_ns
    message.latitude_deg = observation.latitude_deg
    message.longitude_deg = observation.longitude_deg
    message.altitude_m = observation.altitude_m
    message.navsat_status = observation.navsat_status
    message.gnss_fix_ok = observation.gnss_fix_ok
    message.carrier_solution = _CARRIER_TO_CODE[observation.carrier_solution]
    if len(observation.position_covariance) != 9:
        raise ValueError("position covariance must contain exactly 9 ENU values")
    message.position_covariance = list(observation.position_covariance)
    message.covariance_type = int(observation.covariance_type)

    message.utc_stamp_valid = mapping.utc_stamp_valid
    message.utc_seconds = mapping.utc_seconds if mapping.utc_seconds is not None else 0
    message.utc_nanoseconds = mapping.utc_nanoseconds if mapping.utc_nanoseconds is not None else 0
    message.source_time_accuracy_valid = mapping.source_time_accuracy_ns is not None
    message.source_time_accuracy_ns = mapping.source_time_accuracy_ns or 0
    message.leap_seconds_valid = mapping.leap_seconds is not None
    message.leap_seconds = mapping.leap_seconds if mapping.leap_seconds is not None else 0
    message.leap_source_valid = mapping.leap_source is not None
    message.leap_source = mapping.leap_source if mapping.leap_source is not None else 0

    if message.covariance_type != int(CovarianceType.KNOWN):
        raise ValueError("joined UBX NAV-COV must map to known covariance type")
    if message.carrier_solution not in (0, 1, 2, 3):
        raise ValueError("carrier_solution code is outside the ROS contract")
    if mapping.utc_stamp_valid and (
        mapping.utc_seconds is None
        or mapping.utc_nanoseconds is None
        or not 0 <= mapping.utc_nanoseconds < 1_000_000_000
    ):
        raise ValueError("valid UTC mapping requires normalized seconds/nanoseconds")
    if not mapping.utc_stamp_valid and (
        mapping.utc_seconds is not None or mapping.utc_nanoseconds is not None
    ):
        raise ValueError("invalid UTC mapping must not carry timestamp values")
    return message
