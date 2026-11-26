"""Strict conversion from the transport-independent receiver-time mapping to ROSIDL."""


def _integer(name, value, minimum, maximum):
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer in [{minimum}, {maximum}]")
    return value


def _optional_integer(message, valid_field, value_field, value, minimum, maximum):
    setattr(message, valid_field, value is not None)
    setattr(
        message,
        value_field,
        0 if value is None else _integer(value_field, value, minimum, maximum),
    )


def populate_receiver_time_message(mapping, message):
    """Populate a generated GnssReceiverTime-like message, preserving nullability."""
    from robodog_gnss_quality.ros_epoch_adapter import UbxSourceTimeMapping

    if not isinstance(mapping, UbxSourceTimeMapping):
        raise ValueError("mapping must be an UbxSourceTimeMapping")
    _integer("i_tow_ms", mapping.i_tow_ms, 0, 604_799_999)
    _integer("source_timestamp_ns", mapping.source_timestamp_ns, 0, (1 << 64) - 1)
    _integer("receive_monotonic_ns", mapping.receive_monotonic_ns, 0, (1 << 64) - 1)
    _integer("connection_generation", mapping.connection_generation, 0, (1 << 64) - 1)
    if not isinstance(mapping.source_clock_id, str) or not mapping.source_clock_id.strip():
        raise ValueError("source_clock_id must be a non-empty string")
    if not isinstance(mapping.process_session_id, str) or not mapping.process_session_id.strip():
        raise ValueError("process_session_id must be a non-empty string")
    if not isinstance(mapping.valid, bool) or not isinstance(mapping.utc_stamp_valid, bool):
        raise ValueError("mapping validity fields must be booleans")
    if mapping.utc_stamp_valid:
        _integer("utc_seconds", mapping.utc_seconds, 0, (1 << 63) - 1)
        _integer("utc_nanoseconds", mapping.utc_nanoseconds, 0, 999_999_999)
    elif mapping.utc_seconds is not None or mapping.utc_nanoseconds is not None:
        raise ValueError("invalid UTC stamp must not carry timestamp values")
    if not isinstance(mapping.leap_event_valid, bool):
        raise ValueError("leap_event_valid must be a boolean")
    if mapping.leap_event_valid != (mapping.time_to_leap_event_seconds is not None):
        raise ValueError("leap event validity and countdown presence must agree")
    if not isinstance(mapping.validity_reason, str):
        raise ValueError("validity_reason must be a string")

    message.i_tow_ms = mapping.i_tow_ms
    message.source_timestamp_ns = mapping.source_timestamp_ns
    message.source_clock_id = mapping.source_clock_id
    message.receive_monotonic_ns = mapping.receive_monotonic_ns
    message.valid = mapping.valid
    message.utc_stamp_valid = mapping.utc_stamp_valid
    _optional_integer(message, "utc_seconds_valid", "utc_seconds",
                      mapping.utc_seconds, 0, (1 << 63) - 1)
    _optional_integer(message, "utc_nanoseconds_valid", "utc_nanoseconds",
                      mapping.utc_nanoseconds, 0, 999_999_999)
    message.validity_reason = str(mapping.validity_reason)
    _optional_integer(message, "gps_week_valid", "gps_week",
                      mapping.gps_week, 0, 32767)
    _optional_integer(message, "source_time_accuracy_valid", "source_time_accuracy_ns",
                      mapping.source_time_accuracy_ns, 0, (1 << 32) - 1)
    _optional_integer(message, "leap_seconds_valid", "leap_seconds",
                      mapping.leap_seconds, -(1 << 15), (1 << 15) - 1)
    _optional_integer(message, "leap_source_valid", "leap_source",
                      mapping.leap_source, -(1 << 15), (1 << 15) - 1)
    _optional_integer(message, "reported_leap_seconds_valid", "reported_leap_seconds",
                      mapping.reported_leap_seconds, -(1 << 15), (1 << 15) - 1)
    message.leap_event_valid = mapping.leap_event_valid
    _optional_integer(message, "time_to_leap_event_seconds_valid",
                      "time_to_leap_event_seconds", mapping.time_to_leap_event_seconds,
                      -(1 << 31), (1 << 31) - 1)
    message.process_session_id = mapping.process_session_id
    message.connection_generation = mapping.connection_generation
    return message
