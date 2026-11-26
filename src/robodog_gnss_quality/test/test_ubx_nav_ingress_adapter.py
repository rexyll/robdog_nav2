import struct

import pytest

from robodog_gnss_quality.quality_gate import CarrierSolution
from robodog_gnss_quality.ubx_nav_ingress_adapter import (
    decode_ubx_nav_ingress_frame,
    decode_ubx_nav_raw_record,
)
from robodog_gnss_quality.ubx_nav_pvt import EpochPairingError


SESSION = "usb-session-9"
GENERATION = 3
WEEK = 2450
I_TOW = 345678


def ubx_frame(message_id, payload, message_class=0x01):
    header = bytes((0xB5, 0x62, message_class, message_id))
    length = struct.pack("<H", len(payload))
    body = header[2:] + length + payload
    ck_a = 0
    ck_b = 0
    for value in body:
        ck_a = (ck_a + value) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return header + length + payload + bytes((ck_a, ck_b))


def pvt_payload(*, flags=0x81, flags3=0, fix_type=3):
    payload = bytearray(92)
    struct.pack_into("<I", payload, 0, I_TOW)
    payload[20] = fix_type
    payload[21] = flags
    payload[78] = flags3
    return bytes(payload)


def hppos_payload(*, flags=0, version=0):
    payload = bytearray(36)
    payload[0] = version
    payload[3] = flags
    struct.pack_into("<I", payload, 4, I_TOW)
    struct.pack_into("<iiii", payload, 8, -1220000000, 370000000, 15000, 10000)
    struct.pack_into("<bbbb", payload, 24, 1, -1, 2, 0)
    struct.pack_into("<II", payload, 28, 100, 120)
    return bytes(payload)


def cov_payload(*, pos_valid=1, version=0):
    payload = bytearray(64)
    struct.pack_into("<I", payload, 0, I_TOW)
    payload[4] = version
    payload[5] = pos_valid
    payload[6] = 1
    struct.pack_into("<ffffff", payload, 16, 1.0, 0.0, 0.0, 1.5, 0.0, 2.0)
    return bytes(payload)


def status_payload(*, flags=0x0F, fix_stat=0x03, flags2=0x80, fix_type=3):
    payload = bytearray(16)
    struct.pack_into("<I", payload, 0, I_TOW)
    payload[4] = fix_type
    payload[5] = flags
    payload[6] = fix_stat
    payload[7] = flags2
    struct.pack_into("<II", payload, 8, 700, 123456)
    return bytes(payload)


def decode(frame, sequence):
    return decode_ubx_nav_ingress_frame(
        frame,
        protocol=0,
        receive_monotonic_ns=100 + sequence,
        process_session_id=SESSION,
        connection_generation=GENERATION,
        ingress_sequence=sequence,
    )


def test_decode_nav_pvt_raw_frame_and_attach_week_only_at_mapping_boundary():
    raw = decode(ubx_frame(0x07, pvt_payload()), 10)
    assert raw.kind == "pvt"
    assert raw.i_tow_ms == I_TOW
    epoch = decode_ubx_nav_raw_record(raw, gps_week=WEEK)
    assert epoch.gps_week == WEEK
    assert epoch.gnss_fix_ok is True
    assert epoch.carrier_solution is CarrierSolution.FIXED
    assert epoch.process_session_id == SESSION
    assert epoch.connection_generation == GENERATION
    assert epoch.receive_monotonic_ns == 110


def test_decode_hpposllh_preserves_each_invalid_component_bit():
    raw = decode(ubx_frame(0x14, hppos_payload(flags=0x22)), 11)
    epoch = decode_ubx_nav_raw_record(raw, gps_week=WEEK)
    assert epoch.invalid_lon is False
    assert epoch.invalid_lat is False
    assert epoch.invalid_height is False
    assert epoch.invalid_lon_hp is False
    assert epoch.invalid_lat_hp is True
    assert epoch.invalid_height_hp is False
    assert epoch.lon == -1220000000
    assert epoch.lat == 370000000
    assert epoch.height_mm == 15000


def test_decode_nav_cov_reads_position_validity_and_ned_terms():
    raw = decode(ubx_frame(0x36, cov_payload()), 12)
    epoch = decode_ubx_nav_raw_record(raw, gps_week=WEEK)
    assert epoch.pos_cor_valid is True
    assert epoch.pos_cov_nn == pytest.approx(1.0)
    assert epoch.pos_cov_ee == pytest.approx(1.5)
    assert epoch.pos_cov_dd == pytest.approx(2.0)


def test_decode_nav_status_uses_flags2_for_carrier_and_fixstat_for_validity():
    raw = decode(ubx_frame(0x03, status_payload()), 13)
    epoch = decode_ubx_nav_raw_record(raw, gps_week=WEEK)
    assert epoch.gnss_fix_ok is True
    assert epoch.carrier_solution_valid is True
    assert epoch.carrier_solution is CarrierSolution.FIXED


def test_nav_pvt_invalid_llh_and_unsupported_versions_are_rejected():
    raw = decode(ubx_frame(0x07, pvt_payload(flags3=1)), 14)
    with pytest.raises(EpochPairingError, match="LLH invalid"):
        decode_ubx_nav_raw_record(raw, gps_week=WEEK)
    raw_hppos = decode(ubx_frame(0x14, hppos_payload(version=1)), 15)
    with pytest.raises(EpochPairingError, match="unsupported NAV-HPPOSLLH"):
        decode_ubx_nav_raw_record(raw_hppos, gps_week=WEEK)
    raw_cov = decode(ubx_frame(0x36, cov_payload(version=1)), 16)
    with pytest.raises(EpochPairingError, match="unsupported NAV-COV"):
        decode_ubx_nav_raw_record(raw_cov, gps_week=WEEK)


def test_frame_checksum_length_and_context_are_revalidated():
    frame = bytearray(ubx_frame(0x07, pvt_payload()))
    frame[-1] ^= 0x01
    with pytest.raises(EpochPairingError, match="checksum"):
        decode(bytes(frame), 17)
    short = ubx_frame(0x07, pvt_payload())[:-1]
    with pytest.raises(EpochPairingError, match="length mismatch"):
        decode(short, 18)
    with pytest.raises(EpochPairingError, match="connection_generation"):
        decode_ubx_nav_ingress_frame(
            ubx_frame(0x07, pvt_payload()), protocol=0,
            receive_monotonic_ns=1, process_session_id=SESSION,
            connection_generation=True, ingress_sequence=1,
        )


def test_non_nav_and_non_ubx_frames_are_ignored():
    assert decode(ubx_frame(0x20, b"\x00" * 16), 19) is None
    assert decode_ubx_nav_ingress_frame(
        b"not UBX", protocol="nmea", receive_monotonic_ns=1,
        process_session_id=SESSION, connection_generation=GENERATION,
        ingress_sequence=20,
    ) is None


def test_nav_payload_lengths_and_itow_are_exact():
    with pytest.raises(EpochPairingError, match="exactly 92"):
        decode(ubx_frame(0x07, pvt_payload()[:-1]), 21)
    bad = bytearray(pvt_payload())
    struct.pack_into("<I", bad, 0, 604800000)
    with pytest.raises(EpochPairingError, match="iTOW"):
        decode(ubx_frame(0x07, bytes(bad)), 22)
