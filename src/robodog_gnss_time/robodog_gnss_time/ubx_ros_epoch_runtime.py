"""Strict ROSIDL adapters and generation-fenced four-stream GNSS epoch runtime.

ROS callbacks sample host monotonic time locally. ROS Header stamps and host
wall-clock time are never used to derive receiver GPST.
"""

import math
import time
from types import SimpleNamespace

from robodog_gnss_quality.quality_gate import CarrierSolution
from robodog_gnss_quality.ros_epoch_adapter import (
    UbxRosEpochAdapter,
    UbxSourceTimeMapping,
)
from robodog_gnss_quality.ubx_nav_pvt import (
    EpochPairingError,
    UbxNavCovEpoch,
    UbxNavHpPosLlhEpoch,
    UbxNavPvtEpoch,
    UbxNavStatusEpoch,
)


def _integer(name, value, minimum, maximum):
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or value < minimum
        or value > maximum
    ):
        raise EpochPairingError(f"{name} must be an integer in [{minimum}, {maximum}]")
    return value


def _boolean(name, value):
    if not isinstance(value, bool):
        raise EpochPairingError(f"{name} must be a boolean")
    return value


def _text(name, value):
    if not isinstance(value, str) or not value.strip():
        raise EpochPairingError(f"{name} must be a non-empty string")
    return value


def _finite(name, value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EpochPairingError(f"{name} must be a finite number")
    value = float(value)
    if not math.isfinite(value):
        raise EpochPairingError(f"{name} must be a finite number")
    return value


def _context(message):
    return (
        _text("process_session_id", message.process_session_id),
        _integer("connection_generation", message.connection_generation, 0, (1 << 64) - 1),
        _integer("gps_week", message.gps_week, 0, 32767),
    )


def _receive_time(clock):
    return _integer("receive_monotonic_ns", clock(), 0, (1 << 64) - 1)


def _carrier(code):
    return {
        0: CarrierSolution.NO_SOLUTION,
        1: CarrierSolution.FLOAT,
        2: CarrierSolution.FIXED,
        3: CarrierSolution.UNKNOWN,
    }[code]


def nav_pvt_from_ros(message, *, receive_monotonic_ns):
    session, generation, week = _context(message)
    i_tow = _integer("NAV-PVT i_tow_ms", message.i_tow_ms, 0, 604_799_999)
    _integer("NAV-PVT gps_fix_type", message.gps_fix_type, 0, 5)
    gnss_fix_ok = _boolean("NAV-PVT gnss_fix_ok", message.gnss_fix_ok)
    _boolean("NAV-PVT diff_soln", message.diff_soln)
    invalid_llh = _boolean("NAV-PVT invalid_llh", message.invalid_llh)
    if invalid_llh:
        raise EpochPairingError("NAV-PVT marks LLH invalid")
    carrier_code = _integer("NAV-PVT carrier_solution", message.carrier_solution, 0, 3)
    return UbxNavPvtEpoch(
        process_session_id=session,
        connection_generation=generation,
        gps_week=week,
        i_tow_ms=i_tow,
        gnss_fix_ok=gnss_fix_ok,
        carrier_solution=_carrier(carrier_code),
        receive_monotonic_ns=_integer(
            "receive_monotonic_ns", receive_monotonic_ns, 0, (1 << 64) - 1
        ),
    )


def nav_hp_pos_llh_from_ros(message, *, receive_monotonic_ns):
    session, generation, week = _context(message)
    i_tow = _integer("HPPOSLLH i_tow_ms", message.i_tow_ms, 0, 604_799_999)
    version = _integer("HPPOSLLH version", message.version, 0, 255)
    if version != 0:
        raise EpochPairingError("unsupported HPPOSLLH message version")
    invalid_lon = _boolean("HPPOSLLH invalid_lon", message.invalid_lon)
    invalid_lat = _boolean("HPPOSLLH invalid_lat", message.invalid_lat)
    invalid_height = _boolean("HPPOSLLH invalid_height", message.invalid_height)
    invalid_lon_hp = _boolean("HPPOSLLH invalid_lon_hp", message.invalid_lon_hp)
    invalid_lat_hp = _boolean("HPPOSLLH invalid_lat_hp", message.invalid_lat_hp)
    invalid_height_hp = _boolean("HPPOSLLH invalid_height_hp", message.invalid_height_hp)
    _boolean("HPPOSLLH invalid_hmsl", message.invalid_hmsl)
    _boolean("HPPOSLLH invalid_hmsl_hp", message.invalid_hmsl_hp)
    _integer("HPPOSLLH hmsl_mm", message.hmsl_mm, -(1 << 31), (1 << 31) - 1)
    _integer("HPPOSLLH hmsl_hp", message.hmsl_hp, -128, 127)
    _integer("HPPOSLLH h_acc", message.h_acc, 0, (1 << 32) - 1)
    _integer("HPPOSLLH v_acc", message.v_acc, 0, (1 << 32) - 1)
    return UbxNavHpPosLlhEpoch(
        process_session_id=session,
        connection_generation=generation,
        gps_week=week,
        i_tow_ms=i_tow,
        lon=_integer("HPPOSLLH lon", message.lon, -(1 << 31), (1 << 31) - 1),
        lat=_integer("HPPOSLLH lat", message.lat, -(1 << 31), (1 << 31) - 1),
        height_mm=_integer("HPPOSLLH height_mm", message.height_mm, -(1 << 31), (1 << 31) - 1),
        lon_hp=_integer("HPPOSLLH lon_hp", message.lon_hp, -128, 127),
        lat_hp=_integer("HPPOSLLH lat_hp", message.lat_hp, -128, 127),
        height_hp=_integer("HPPOSLLH height_hp", message.height_hp, -128, 127),
        invalid_lon=invalid_lon,
        invalid_lat=invalid_lat,
        invalid_height=invalid_height,
        invalid_lon_hp=invalid_lon_hp,
        invalid_lat_hp=invalid_lat_hp,
        invalid_height_hp=invalid_height_hp,
        receive_monotonic_ns=_integer(
            "receive_monotonic_ns", receive_monotonic_ns, 0, (1 << 64) - 1
        ),
    )


def nav_cov_from_ros(message, *, receive_monotonic_ns):
    session, generation, week = _context(message)
    i_tow = _integer("NAV-COV i_tow_ms", message.i_tow_ms, 0, 604_799_999)
    version = _integer("NAV-COV version", message.version, 0, 255)
    if version != 0:
        raise EpochPairingError("unsupported NAV-COV message version")
    pos_cor_valid = _boolean("NAV-COV pos_cor_valid", message.pos_cor_valid)
    _boolean("NAV-COV vel_cor_valid", message.vel_cor_valid)
    return UbxNavCovEpoch(
        process_session_id=session,
        connection_generation=generation,
        gps_week=week,
        i_tow_ms=i_tow,
        pos_cor_valid=pos_cor_valid,
        pos_cov_nn=_finite("NAV-COV pos_cov_nn", message.pos_cov_nn),
        pos_cov_ne=_finite("NAV-COV pos_cov_ne", message.pos_cov_ne),
        pos_cov_nd=_finite("NAV-COV pos_cov_nd", message.pos_cov_nd),
        pos_cov_ee=_finite("NAV-COV pos_cov_ee", message.pos_cov_ee),
        pos_cov_ed=_finite("NAV-COV pos_cov_ed", message.pos_cov_ed),
        pos_cov_dd=_finite("NAV-COV pos_cov_dd", message.pos_cov_dd),
        receive_monotonic_ns=_integer(
            "receive_monotonic_ns", receive_monotonic_ns, 0, (1 << 64) - 1
        ),
    )


def nav_status_from_ros(message, *, receive_monotonic_ns):
    session, generation, week = _context(message)
    i_tow = _integer("NAV-STATUS i_tow_ms", message.i_tow_ms, 0, 604_799_999)
    _integer("NAV-STATUS gps_fix_type", message.gps_fix_type, 0, 5)
    gnss_fix_ok = _boolean("NAV-STATUS gps_fix_ok", message.gps_fix_ok)
    _boolean("NAV-STATUS diff_soln", message.diff_soln)
    _boolean("NAV-STATUS wkn_set", message.wkn_set)
    _boolean("NAV-STATUS tow_set", message.tow_set)
    _boolean("NAV-STATUS diff_corr", message.diff_corr)
    carrier_solution_valid = _boolean(
        "NAV-STATUS carrier_solution_valid", message.carrier_solution_valid
    )
    carrier_code = _integer("NAV-STATUS carrier_solution", message.carrier_solution, 0, 3)
    _integer("NAV-STATUS map_matching_status", message.map_matching_status, 0, 3)
    _integer("NAV-STATUS psm_state", message.psm_state, 0, 3)
    _integer("NAV-STATUS spoof_det_state", message.spoof_det_state, 0, 3)
    _integer("NAV-STATUS ttff", message.ttff, 0, (1 << 32) - 1)
    _integer("NAV-STATUS msss", message.msss, 0, (1 << 32) - 1)
    return UbxNavStatusEpoch(
        process_session_id=session,
        connection_generation=generation,
        gps_week=week,
        i_tow_ms=i_tow,
        gnss_fix_ok=gnss_fix_ok,
        carrier_solution_valid=carrier_solution_valid,
        carrier_solution=_carrier(carrier_code),
        receive_monotonic_ns=_integer(
            "receive_monotonic_ns", receive_monotonic_ns, 0, (1 << 64) - 1
        ),
    )


def _upstream_context_message(message, *, process_session_id, connection_generation,
                              gps_week, receive_monotonic_ns, **fields):
    """Normalize upstream ublox_ubx_msgs fields without using Header.stamp."""
    return SimpleNamespace(
        i_tow_ms=message.itow,
        process_session_id=process_session_id,
        connection_generation=connection_generation,
        gps_week=gps_week,
        **fields,
    )


def nav_pvt_from_ublox_ros(
    message, *, process_session_id, connection_generation, gps_week,
    receive_monotonic_ns,
):
    normalized = _upstream_context_message(
        message, process_session_id=process_session_id,
        connection_generation=connection_generation, gps_week=gps_week,
        receive_monotonic_ns=receive_monotonic_ns,
        gps_fix_type=message.gps_fix.fix_type,
        gnss_fix_ok=message.gnss_fix_ok,
        diff_soln=message.diff_soln,
        carrier_solution=message.carr_soln.status,
        invalid_llh=message.invalid_llh,
    )
    return nav_pvt_from_ros(normalized, receive_monotonic_ns=receive_monotonic_ns)


def nav_hp_pos_llh_from_ublox_ros(
    message, *, process_session_id, connection_generation, gps_week,
    receive_monotonic_ns,
):
    normalized = _upstream_context_message(
        message, process_session_id=process_session_id,
        connection_generation=connection_generation, gps_week=gps_week,
        receive_monotonic_ns=receive_monotonic_ns,
        version=message.version,
        invalid_lon=message.invalid_lon, invalid_lat=message.invalid_lat,
        invalid_height=message.invalid_height, invalid_hmsl=message.invalid_hmsl,
        invalid_lon_hp=message.invalid_lon_hp, invalid_lat_hp=message.invalid_lat_hp,
        invalid_height_hp=message.invalid_height_hp, invalid_hmsl_hp=message.invalid_hmsl_hp,
        lon=message.lon, lat=message.lat, height_mm=message.height,
        hmsl_mm=message.hmsl, lon_hp=message.lon_hp, lat_hp=message.lat_hp,
        height_hp=message.height_hp, hmsl_hp=message.hmsl_hp,
        h_acc=message.h_acc, v_acc=message.v_acc,
    )
    return nav_hp_pos_llh_from_ros(normalized, receive_monotonic_ns=receive_monotonic_ns)


def nav_cov_from_ublox_ros(
    message, *, process_session_id, connection_generation, gps_week,
    receive_monotonic_ns,
):
    normalized = _upstream_context_message(
        message, process_session_id=process_session_id,
        connection_generation=connection_generation, gps_week=gps_week,
        receive_monotonic_ns=receive_monotonic_ns,
        version=message.version, pos_cor_valid=message.pos_cor_valid,
        vel_cor_valid=message.vel_cor_valid,
        pos_cov_nn=message.pos_cov_nn, pos_cov_ne=message.pos_cov_ne,
        pos_cov_nd=message.pos_cov_nd, pos_cov_ee=message.pos_cov_ee,
        pos_cov_ed=message.pos_cov_ed, pos_cov_dd=message.pos_cov_dd,
    )
    return nav_cov_from_ros(normalized, receive_monotonic_ns=receive_monotonic_ns)


def nav_status_from_ublox_ros(
    message, *, process_session_id, connection_generation, gps_week,
    receive_monotonic_ns,
):
    normalized = _upstream_context_message(
        message, process_session_id=process_session_id,
        connection_generation=connection_generation, gps_week=gps_week,
        receive_monotonic_ns=receive_monotonic_ns,
        gps_fix_type=message.gps_fix.fix_type,
        gps_fix_ok=message.gps_fix_ok, diff_soln=message.diff_soln,
        wkn_set=message.wkn_set, tow_set=message.tow_set,
        diff_corr=message.diff_corr, carrier_solution_valid=message.carr_soln_valid,
        carrier_solution=message.carr_soln.status,
        map_matching_status=message.map_matching.status,
        psm_state=message.psm.state, spoof_det_state=message.spoof_det.state,
        ttff=message.ttff, msss=message.msss,
    )
    return nav_status_from_ros(normalized, receive_monotonic_ns=receive_monotonic_ns)


def _optional_from_ros(message, valid_name, value_name, minimum, maximum):
    valid = _boolean(valid_name, getattr(message, valid_name))
    value = _integer(value_name, getattr(message, value_name), minimum, maximum)
    if not valid:
        if value != 0:
            raise EpochPairingError(f"{value_name} must be zero when {valid_name}=false")
        return None
    return value


def receiver_time_from_ros(message):
    """Convert GnssReceiverTime ROSIDL, checking every validity/sentinel pair."""
    valid = _boolean("valid", message.valid)
    utc_valid = _boolean("utc_stamp_valid", message.utc_stamp_valid)
    utc_seconds = _optional_from_ros(
        message, "utc_seconds_valid", "utc_seconds", 0, (1 << 63) - 1
    )
    utc_nanoseconds = _optional_from_ros(
        message, "utc_nanoseconds_valid", "utc_nanoseconds", 0, 999_999_999
    )
    if utc_valid != (utc_seconds is not None and utc_nanoseconds is not None):
        raise EpochPairingError("utc_stamp_valid must agree with UTC seconds/nanoseconds validity")
    if not utc_valid and (utc_seconds is not None or utc_nanoseconds is not None):
        raise EpochPairingError("invalid UTC stamp must not carry timestamp values")

    gps_week = _optional_from_ros(message, "gps_week_valid", "gps_week", 0, 32767)
    accuracy = _optional_from_ros(
        message, "source_time_accuracy_valid", "source_time_accuracy_ns", 0, (1 << 32) - 1
    )
    leap_seconds = _optional_from_ros(
        message, "leap_seconds_valid", "leap_seconds", -(1 << 15), (1 << 15) - 1
    )
    leap_source = _optional_from_ros(
        message, "leap_source_valid", "leap_source", -(1 << 15), (1 << 15) - 1
    )
    reported_leap = _optional_from_ros(
        message, "reported_leap_seconds_valid", "reported_leap_seconds",
        -(1 << 15), (1 << 15) - 1
    )
    event_valid = _boolean("leap_event_valid", message.leap_event_valid)
    event_countdown = _optional_from_ros(
        message, "time_to_leap_event_seconds_valid",
        "time_to_leap_event_seconds", -(1 << 31), (1 << 31) - 1
    )
    if event_valid != (event_countdown is not None):
        raise EpochPairingError("leap event validity must agree with countdown presence")

    mapping = UbxSourceTimeMapping(
        i_tow_ms=_integer("i_tow_ms", message.i_tow_ms, 0, 604_799_999),
        source_timestamp_ns=_integer("source_timestamp_ns", message.source_timestamp_ns, 0, (1 << 64) - 1),
        source_clock_id=_text("source_clock_id", message.source_clock_id),
        receive_monotonic_ns=_integer("receive_monotonic_ns", message.receive_monotonic_ns, 0, (1 << 64) - 1),
        valid=valid,
        utc_stamp_valid=utc_valid,
        utc_seconds=utc_seconds,
        utc_nanoseconds=utc_nanoseconds,
        validity_reason=message.validity_reason,
        gps_week=gps_week,
        source_time_accuracy_ns=accuracy,
        leap_seconds=leap_seconds,
        leap_source=leap_source,
        reported_leap_seconds=reported_leap,
        leap_event_valid=event_valid,
        time_to_leap_event_seconds=event_countdown,
        process_session_id=_text("process_session_id", message.process_session_id),
        connection_generation=_integer(
            "connection_generation", message.connection_generation, 0, (1 << 64) - 1
        ),
    )
    if not isinstance(mapping.validity_reason, str):
        raise EpochPairingError("validity_reason must be a string")
    if mapping.valid and mapping.gps_week is None:
        raise EpochPairingError("valid receiver-time mapping requires a valid GPS week")
    return mapping


class UbxRosEpochRuntime:
    """Generation-fenced ROS-message source adapter around UbxRosEpochAdapter.

    Before the first mapping, decoded navigation records may be cached under
    their supplied session/generation/week. They cannot produce output without
    a valid matching receiver-time mapping. A changed generation is activated
    only by a valid receiver-time mapping; raw records from other generations
    are discarded rather than allowed to reset the trust boundary.
    """

    def __init__(
        self,
        *,
        max_pending_epochs,
        max_pair_receive_skew_ns,
        max_pending_age_ns,
        publish_observation,
        monotonic_clock=time.monotonic_ns,
    ):
        if not callable(publish_observation) or not callable(monotonic_clock):
            raise ValueError("publish_observation and monotonic_clock must be callable")
        self.adapter = UbxRosEpochAdapter(
            max_pending_epochs=max_pending_epochs,
            max_pair_receive_skew_ns=max_pair_receive_skew_ns,
            max_pending_age_ns=max_pending_age_ns,
        )
        self._publish_observation = publish_observation
        self._monotonic_clock = monotonic_clock
        self._active_context = None
        self._stale_context_drops = 0

    @property
    def stale_context_drops(self):
        return self._stale_context_drops

    def _activate(self, context):
        session, generation, week = context
        source_clock_id = f"{session}:connection:{generation}:gps-week:{week}"
        self.adapter.begin_generation(
            source_clock_id,
            process_session_id=session,
            connection_generation=generation,
            gps_week=week,
        )
        self._active_context = context

    def _handle_raw(self, message, converter, add_method):
        receive_ns = _receive_time(self._monotonic_clock)
        record = converter(message, receive_monotonic_ns=receive_ns)
        context = (record.process_session_id, record.connection_generation, record.gps_week)
        if self._active_context is None:
            self._activate(context)
        elif context != self._active_context:
            self._stale_context_drops += 1
            return None
        result = getattr(self.adapter, add_method)(record)
        self._publish(result)
        return result

    def handle_nav_pvt(self, message):
        return self._handle_raw(message, nav_pvt_from_ros, "add_pvt")

    def handle_nav_hp_pos_llh(self, message):
        return self._handle_raw(message, nav_hp_pos_llh_from_ros, "add_position")

    def handle_nav_cov(self, message):
        return self._handle_raw(message, nav_cov_from_ros, "add_covariance")

    def handle_nav_status(self, message):
        return self._handle_raw(message, nav_status_from_ros, "add_status")

    def handle_receiver_time(self, message):
        mapping = receiver_time_from_ros(message)
        if not mapping.valid:
            # Without a valid GPS week there is no safe epoch/generation key.
            # Invalid mappings with a complete active context can retire only
            # their exact iTOW; incomplete context is discarded, not guessed.
            if self._active_context is not None and mapping.gps_week is not None:
                context = (
                    mapping.process_session_id,
                    mapping.connection_generation,
                    mapping.gps_week,
                )
                if context == self._active_context:
                    result = self.adapter.add_time_mapping(mapping)
                    self._publish(result)
                    return result
            return None
        if mapping.gps_week is None:
            raise EpochPairingError("valid receiver-time mapping requires gps_week")
        context = (mapping.process_session_id, mapping.connection_generation, mapping.gps_week)
        if context != self._active_context:
            self._activate(context)
        result = self.adapter.add_time_mapping(mapping)
        self._publish(result)
        return result

    def _publish(self, result):
        if result is not None:
            self._publish_observation(result)
        return result

    def expire_stale(self):
        return self.adapter.expire_stale(_receive_time(self._monotonic_clock))
