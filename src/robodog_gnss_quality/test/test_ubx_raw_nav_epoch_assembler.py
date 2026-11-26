import struct

import pytest

from robodog_gnss_quality.ros_epoch_adapter import UbxSourceTimeMapping
from robodog_gnss_quality.ubx_ingress_event_processor import GnssIngressEvent
from robodog_gnss_quality.ubx_raw_nav_epoch_assembler import UbxRawNavEpochAssembler


SESSION = "raw-usb-session"
GENERATION = 2
WEEK = 2450
I_TOW = 234567


def ubx_frame(message_id, payload):
    header = bytes((0xB5, 0x62, 0x01, message_id))
    length = struct.pack("<H", len(payload))
    body = header[2:] + length + payload
    a = b = 0
    for value in body:
        a = (a + value) & 0xFF
        b = (b + a) & 0xFF
    return header + length + payload + bytes((a, b))


def pvt_payload(itow=I_TOW, flags=0x81):
    p = bytearray(92)
    struct.pack_into("<I", p, 0, itow)
    p[20] = 3
    p[21] = flags
    p[78] = 0
    return bytes(p)


def hppos_payload(itow=I_TOW):
    p = bytearray(36)
    p[0] = 0
    struct.pack_into("<I", p, 4, itow)
    struct.pack_into("<iiii", p, 8, -1220000000, 370000000, 15000, 10000)
    struct.pack_into("<bbbb", p, 24, 1, -1, 2, 0)
    struct.pack_into("<II", p, 28, 100, 120)
    return bytes(p)


def cov_payload(itow=I_TOW):
    p = bytearray(64)
    struct.pack_into("<I", p, 0, itow)
    p[4] = 0
    p[5] = 1
    p[6] = 1
    struct.pack_into("<ffffff", p, 16, 1.0, 0.0, 0.0, 1.5, 0.0, 2.0)
    return bytes(p)


def status_payload(itow=I_TOW):
    p = bytearray(16)
    struct.pack_into("<I", p, 0, itow)
    p[4] = 3
    p[5] = 0x0F
    p[6] = 0x03
    p[7] = 0x80
    return bytes(p)


def lifecycle(kind, sequence, receive_ns, session=SESSION, generation=GENERATION):
    return GnssIngressEvent(
        event_type=kind,
        process_session_id=session,
        connection_generation=generation,
        ingress_sequence=sequence,
        receive_monotonic_ns=receive_ns,
    )


def frame_event(message_id, payload, sequence, receive_ns, session=SESSION, generation=GENERATION):
    return GnssIngressEvent(
        event_type="Frame",
        process_session_id=session,
        connection_generation=generation,
        ingress_sequence=sequence,
        receive_monotonic_ns=receive_ns,
        protocol=0,
        checksum_status=1,
        frame_sequence=sequence,
        frame_connection_generation=generation,
        frame_bytes=ubx_frame(message_id, payload),
    )


def mapping(i_tow=I_TOW, receive_ns=1450, session=SESSION, generation=GENERATION, week=WEEK):
    return UbxSourceTimeMapping(
        i_tow_ms=i_tow,
        source_timestamp_ns=1_480_000_000_000_000_000 + i_tow * 1_000_000,
        source_clock_id=f"{session}:connection:{generation}:gps-week:{week}",
        receive_monotonic_ns=receive_ns,
        valid=True,
        utc_stamp_valid=False,
        validity_reason="UTC not trusted",
        gps_week=week,
        source_time_accuracy_ns=40,
        leap_seconds=18,
        leap_source=2,
        reported_leap_seconds=18,
        leap_event_valid=False,
        process_session_id=session,
        connection_generation=generation,
    )


def assembler(**overrides):
    args = dict(
        max_pending_epochs=8,
        max_pair_receive_skew_ns=500,
        max_pending_age_ns=10_000,
    )
    args.update(overrides)
    return UbxRawNavEpochAssembler(**args)


def start(a):
    a.process_event(lifecycle("GenerationStarted", 0, 100))


def send_four(a, first_sequence=1, first_receive=1100, itow=I_TOW):
    records = [
        (0x07, pvt_payload(itow), first_sequence, first_receive),
        (0x14, hppos_payload(itow), first_sequence + 1, first_receive + 10),
        (0x36, cov_payload(itow), first_sequence + 2, first_receive + 20),
        (0x03, status_payload(itow), first_sequence + 3, first_receive + 30),
    ]
    results = []
    for message_id, payload, sequence, receive_ns in records:
        results.append(a.process_event(frame_event(message_id, payload, sequence, receive_ns)))
    return results


def test_raw_four_streams_buffer_until_exact_receiver_time_mapping_arrives():
    a = assembler()
    start(a)
    assert all(result is None for result in send_four(a))
    assert a.stats().buffered_before_mapping == 4
    result = a.handle_mapping(mapping())
    assert result is not None
    assert result.observation.source_timestamp_ns == mapping().source_timestamp_ns
    assert result.observation.latitude_deg == pytest.approx(37.0, abs=2e-8)
    assert result.observation.longitude_deg == pytest.approx(-122.0, abs=2e-8)
    assert result.observation.gnss_fix_ok is True
    assert a.stats().emitted_observations == 1


def test_receiver_time_mapping_can_arrive_before_raw_nav_frames():
    a = assembler()
    start(a)
    assert a.handle_mapping(mapping(receive_ns=1000)) is None
    results = send_four(a, first_sequence=1, first_receive=1010)
    assert all(result is None for result in results[:3])
    assert results[3] is not None
    assert results[3].observation.source_clock_id == mapping().source_clock_id


def test_exact_itow_is_required_and_unmatched_epoch_remains_bounded():
    a = assembler()
    start(a)
    send_four(a)
    assert a.handle_mapping(mapping(i_tow=I_TOW + 1, receive_ns=1450)) is None
    assert a.adapter.pending_count == 1
    result = a.handle_mapping(mapping(i_tow=I_TOW, receive_ns=1451))
    assert result is not None


def test_transport_generation_fences_old_time_mapping():
    a = assembler()
    start(a)
    a.process_event(lifecycle("DeviceLost", 1, 200))
    a.process_event(lifecycle("GenerationStarted", 2, 300, generation=GENERATION + 1))
    stale = mapping(receive_ns=400, generation=GENERATION)
    assert a.handle_mapping(stale) is None
    assert a.stats().rejected_stale_generation == 1


def test_conflicting_duplicate_raw_message_retires_epoch():
    a = assembler()
    start(a)
    a.process_event(frame_event(0x07, pvt_payload(flags=0x81), 1, 1100))
    a.process_event(frame_event(0x07, pvt_payload(flags=0x01), 2, 1110))
    assert a.stats().conflicting_epochs == 1
    assert a.handle_mapping(mapping()) is None
    # A later duplicate cannot resurrect a retired iTOW in this generation.
    assert a.process_event(frame_event(0x07, pvt_payload(), 3, 1120)) is None
    assert a.stats().decoded_nav_frames == 3


def test_mapping_receive_skew_rejects_stale_raw_records():
    a = assembler(max_pair_receive_skew_ns=100)
    start(a)
    send_four(a, first_receive=1000)
    assert a.handle_mapping(mapping(receive_ns=5000)) is None
    assert a.stats().rejected_receive_skew == 4


def test_raw_pending_epochs_expire_and_capacity_is_bounded():
    a = assembler(max_pending_epochs=1, max_pending_age_ns=100)
    start(a)
    a.process_event(frame_event(0x07, pvt_payload(I_TOW), 1, 110))
    a.process_event(frame_event(0x07, pvt_payload(I_TOW + 1), 2, 120))
    assert a.stats().capacity_evictions == 1
    a.expire_stale(1000)
    assert a.stats().expired_raw_epochs == 1


def test_expired_epoch_is_retired_and_late_duplicate_cannot_resurrect():
    a = assembler(max_pending_age_ns=100)
    start(a)
    a.process_event(frame_event(0x07, pvt_payload(), 1, 110))
    a.expire_stale(1000)
    assert a.stats().expired_raw_epochs == 1
    a.process_event(frame_event(0x07, pvt_payload(), 2, 1010))
    assert a.stats().buffered_before_mapping == 1
    assert a.handle_mapping(mapping(receive_ns=1020)) is None
