"""Strict ROSIDL adapter for the receiver-time mapping wire contract.

This module stays transport-independent: it accepts a generated-message-like
object and returns the immutable UbxSourceTimeMapping used by the epoch joiner.
"""

from .ros_epoch_adapter import UbxSourceTimeMapping
from .ubx_nav_pvt import EpochPairingError, UBX_GPS_WEEK_MS


def _integer(name, value, minimum, maximum):
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise EpochPairingError(f"{name} must be an integer in [{minimum}, {maximum}]")
    return value


def _boolean(name, value):
    if not isinstance(value, bool):
        raise EpochPairingError(f"{name} must be a boolean")
    return value


def _optional_integer(message, flag_name, value_name, minimum, maximum):
    valid = _boolean(flag_name, getattr(message, flag_name))
    value = _integer(value_name, getattr(message, value_name), minimum, maximum)
    if not valid:
        if value != 0:
            raise EpochPairingError(f"{value_name} must use zero sentinel when {flag_name}=false")
        return None
    return value


def decode_receiver_time_mapping_message(message):
    """Decode GnssReceiverTime, preserving optionality and source-clock identity."""
    try:
        i_tow_ms = _integer("i_tow_ms", message.i_tow_ms, 0, UBX_GPS_WEEK_MS - 1)
        source_timestamp_ns = _integer(
            "source_timestamp_ns", message.source_timestamp_ns, 0, (1 << 64) - 1
        )
        source_clock_id = message.source_clock_id
        receive_monotonic_ns = _integer(
            "receive_monotonic_ns", message.receive_monotonic_ns, 0, (1 << 64) - 1
        )
        valid = _boolean("valid", message.valid)
        utc_stamp_valid = _boolean("utc_stamp_valid", message.utc_stamp_valid)
        utc_seconds = _optional_integer(
            message, "utc_seconds_valid", "utc_seconds", 0, (1 << 63) - 1
        )
        utc_nanoseconds = _optional_integer(
            message, "utc_nanoseconds_valid", "utc_nanoseconds", 0, 999_999_999
        )
        validity_reason = message.validity_reason
        gps_week = _optional_integer(message, "gps_week_valid", "gps_week", 0, 32767)
        source_time_accuracy_ns = _optional_integer(
            message, "source_time_accuracy_valid", "source_time_accuracy_ns",
            0, (1 << 32) - 1
        )
        leap_seconds = _optional_integer(
            message, "leap_seconds_valid", "leap_seconds", -(1 << 15), (1 << 15) - 1
        )
        leap_source = _optional_integer(
            message, "leap_source_valid", "leap_source", -(1 << 15), (1 << 15) - 1
        )
        reported_leap_seconds = _optional_integer(
            message, "reported_leap_seconds_valid", "reported_leap_seconds",
            -(1 << 15), (1 << 15) - 1
        )
        leap_event_valid = _boolean("leap_event_valid", message.leap_event_valid)
        time_to_leap_event_seconds = _optional_integer(
            message, "time_to_leap_event_seconds_valid",
            "time_to_leap_event_seconds", -(1 << 31), (1 << 31) - 1
        )
        process_session_id = message.process_session_id
        connection_generation = _integer(
            "connection_generation", message.connection_generation, 0, (1 << 64) - 1
        )
    except AttributeError as exc:
        raise EpochPairingError("GnssReceiverTime message is missing a required field") from exc

    if not isinstance(source_clock_id, str) or not source_clock_id.strip():
        raise EpochPairingError("source_clock_id must be a non-empty string")
    if not isinstance(validity_reason, str):
        raise EpochPairingError("validity_reason must be a string")
    if not isinstance(process_session_id, str) or not process_session_id.strip():
        raise EpochPairingError("process_session_id must be a non-empty string")
    if valid and gps_week is None:
        raise EpochPairingError("valid receiver-time mapping requires gps_week")
    if gps_week is not None:
        expected_clock_id = (
            f"{process_session_id}:connection:{connection_generation}:gps-week:{gps_week}"
        )
        if source_clock_id != expected_clock_id:
            raise EpochPairingError("source_clock_id does not match session/generation/gps_week")
    if utc_stamp_valid:
        if not valid or utc_seconds is None or utc_nanoseconds is None:
            raise EpochPairingError("valid UTC stamp requires a valid mapping and both UTC fields")
    elif utc_seconds is not None or utc_nanoseconds is not None:
        raise EpochPairingError("invalid UTC stamp must not carry UTC timestamp values")
    if leap_event_valid != (time_to_leap_event_seconds is not None):
        raise EpochPairingError("leap event validity and countdown presence must agree")

    return UbxSourceTimeMapping(
        i_tow_ms=i_tow_ms,
        source_timestamp_ns=source_timestamp_ns,
        source_clock_id=source_clock_id,
        receive_monotonic_ns=receive_monotonic_ns,
        valid=valid,
        utc_stamp_valid=utc_stamp_valid,
        utc_seconds=utc_seconds,
        utc_nanoseconds=utc_nanoseconds,
        validity_reason=validity_reason,
        gps_week=gps_week,
        source_time_accuracy_ns=source_time_accuracy_ns,
        leap_seconds=leap_seconds,
        leap_source=leap_source,
        reported_leap_seconds=reported_leap_seconds,
        leap_event_valid=leap_event_valid,
        time_to_leap_event_seconds=time_to_leap_event_seconds,
        process_session_id=process_session_id,
        connection_generation=connection_generation,
    )
