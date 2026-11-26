import unittest
from concurrent.futures import ThreadPoolExecutor

from robodog_gnss_quality.ubx_nav_pvt import EpochPairingError
from robodog_gnss_quality.ubx_time_mapper import (
    GPS_EPOCH_UNIX_NS,
    GPS_WEEK_NS,
    UbxReceiverTimeMapper,
)
from robodog_gnss_quality.ubx_time_messages import (
    UbxNavTimeGpsEpoch,
    UbxNavTimeLsEpoch,
    decode_nav_timegps_payload,
    decode_nav_timels_payload,
)


SESSION_ID = "f9p-process-session-17"


def timegps_payload(i_tow=1000, f_tow=250_000, week=2450, leap=18, valid=0x07, tacc=50):
    return (
        i_tow.to_bytes(4, "little")
        + f_tow.to_bytes(4, "little", signed=True)
        + week.to_bytes(2, "little", signed=True)
        + leap.to_bytes(1, "little", signed=True)
        + bytes([valid])
        + tacc.to_bytes(4, "little")
    )


def timels_payload(
    i_tow=1000, source=2, leap=18, valid=0x03, time_to_event=86_400, change=1
):
    payload = bytearray(24)
    payload[0:4] = i_tow.to_bytes(4, "little")
    payload[4] = 0
    payload[8] = source
    payload[9:10] = leap.to_bytes(1, "little", signed=True)
    payload[10] = 2
    payload[11:12] = change.to_bytes(1, "little", signed=True)
    payload[12:16] = time_to_event.to_bytes(4, "little", signed=True)
    payload[16:18] = (2451).to_bytes(2, "little")
    payload[18:20] = (7).to_bytes(2, "little")
    payload[23] = valid
    return bytes(payload)


def gps_epoch(
    *, receive=100, process_session_id=SESSION_ID, generation=0, **fields
) -> UbxNavTimeGpsEpoch:
    return decode_nav_timegps_payload(
        timegps_payload(**fields),
        receive_monotonic_ns=receive,
        process_session_id=process_session_id,
        connection_generation=generation,
    )


def ls_epoch(
    *, receive=110, process_session_id=SESSION_ID, generation=0, **fields
) -> UbxNavTimeLsEpoch:
    return decode_nav_timels_payload(
        timels_payload(**fields),
        receive_monotonic_ns=receive,
        process_session_id=process_session_id,
        connection_generation=generation,
    )


def mapper(**kwargs):
    connection_generation = kwargs.pop("connection_generation", 0)
    result = UbxReceiverTimeMapper(**kwargs)
    result.begin_session(SESSION_ID, connection_generation)
    return result


def add_gps(m, receive=100, **fields):
    return m.add_timegps(
        gps_epoch(
            receive=receive,
            process_session_id=m.process_session_id,
            generation=m.connection_generation,
            **fields,
        )
    )


def add_ls(m, receive=110, **fields):
    return m.add_timels(
        ls_epoch(
            receive=receive,
            process_session_id=m.process_session_id,
            generation=m.connection_generation,
            **fields,
        )
    )


class TestUbxTimeMapper(unittest.TestCase):
    def test_pair_in_either_order_and_convert_utc_only_with_trusted_source(self):
        m = mapper(trusted_leap_sources={2})
        self.assertIsNone(add_gps(m, receive=1_000))
        result = add_ls(m, receive=1_100)
        self.assertIsNotNone(result)
        expected_gps_ns = 2450 * GPS_WEEK_NS + 1_000_250_000
        self.assertEqual(result.source_timestamp_ns, expected_gps_ns)
        self.assertTrue(result.valid)
        self.assertTrue(result.utc_stamp_valid)
        self.assertEqual(result.gps_week, 2450)
        expected_utc_ns = GPS_EPOCH_UNIX_NS + expected_gps_ns - 18_000_000_000
        self.assertEqual(result.utc_seconds, expected_utc_ns // 1_000_000_000)
        self.assertEqual(result.utc_nanoseconds, expected_utc_ns % 1_000_000_000)
        self.assertEqual(result.process_session_id, SESSION_ID)
        self.assertEqual(result.connection_generation, 0)
        self.assertEqual(result.source_time_accuracy_ns, 50)

        m2 = mapper(trusted_leap_sources={2})
        self.assertIsNone(add_ls(m2, receive=1_100))
        result2 = add_gps(m2, receive=1_000)
        self.assertEqual(result2, result)

    def test_untrusted_or_invalid_leap_source_keeps_gps_time_but_not_utc(self):
        m = mapper()
        self.assertIsNone(add_gps(m))
        result = add_ls(m, source=2)
        self.assertTrue(result.valid)
        self.assertFalse(result.utc_stamp_valid)
        self.assertIsNone(result.utc_seconds)
        self.assertIn("not trusted", result.validity_reason)

        m2 = mapper(trusted_leap_sources={2})
        add_gps(m2)
        mismatch = add_ls(m2, leap=17)
        self.assertTrue(mismatch.valid)
        self.assertFalse(mismatch.utc_stamp_valid)
        self.assertIn("differs", mismatch.validity_reason)

    def test_beidou_leap_source_is_adjusted_to_gps_utc_before_cross_check(self):
        m = mapper(trusted_leap_sources={4})
        add_gps(m, leap=18)
        result = add_ls(m, source=4, leap=4)
        self.assertTrue(result.utc_stamp_valid)
        self.assertEqual(result.leap_seconds, 18)
        self.assertEqual(result.reported_leap_seconds, 4)

    def test_current_leap_validity_is_independent_of_event_schedule_and_guard(self):
        m = mapper(trusted_leap_sources={2})
        add_gps(m)
        no_event_time = add_ls(m, valid=0x01, change=2)
        self.assertTrue(no_event_time.valid)
        self.assertTrue(no_event_time.utc_stamp_valid)
        self.assertFalse(no_event_time.leap_event_valid)
        self.assertIsNone(no_event_time.time_to_leap_event_seconds)

        for offset in (-1, 0, 1):
            guarded = mapper(trusted_leap_sources={2})
            add_gps(guarded)
            result = add_ls(guarded, time_to_event=offset)
            self.assertTrue(result.valid)
            self.assertFalse(result.utc_stamp_valid)
            self.assertIn("transition guard window", result.validity_reason)

        outside = mapper(trusted_leap_sources={2})
        add_gps(outside)
        result = add_ls(outside, time_to_event=2)
        self.assertTrue(result.utc_stamp_valid)

    def test_firmware_default_leap_source_cannot_be_allowlisted(self):
        with self.assertRaisesRegex(ValueError, "firmware default 0 is never trusted"):
            UbxReceiverTimeMapper(trusted_leap_sources={0})

    def test_utc_stamp_is_withheld_near_valid_leap_event(self):
        m = mapper(trusted_leap_sources={2})
        add_gps(m)
        result = add_ls(m, time_to_event=1)
        self.assertTrue(result.valid)
        self.assertFalse(result.utc_stamp_valid)
        self.assertTrue(result.leap_event_valid)
        self.assertEqual(result.time_to_leap_event_seconds, 1)
        self.assertIn("transition guard window", result.validity_reason)

    def test_negative_or_unrepresentable_gps_utc_leap_offset_blocks_utc(self):
        m = mapper(trusted_leap_sources={2})
        add_gps(m, leap=-1)
        result = add_ls(m, leap=-1)
        self.assertTrue(result.valid)
        self.assertFalse(result.utc_stamp_valid)
        self.assertIn("outside 0..127", result.validity_reason)

    def test_invalid_future_event_fields_do_not_invalidate_current_leap(self):
        m = mapper(trusted_leap_sources={2})
        add_gps(m)
        result = add_ls(m, valid=0x01, change=2)
        self.assertTrue(result.utc_stamp_valid)
        self.assertIsNone(result.time_to_leap_event_seconds)

    def test_timegps_leap_flag_and_timels_validity_are_both_required(self):
        m = mapper(trusted_leap_sources={2})
        add_gps(m, valid=0x03)
        result = add_ls(m)
        self.assertTrue(result.valid)
        self.assertFalse(result.utc_stamp_valid)

        m2 = mapper(trusted_leap_sources={2})
        add_gps(m2)
        result2 = add_ls(m2, valid=0x02)
        self.assertFalse(result2.utc_stamp_valid)

    def test_timegps_and_timels_must_match_exact_itow(self):
        m = mapper(trusted_leap_sources={2})
        self.assertIsNone(add_gps(m, i_tow=1000))
        self.assertIsNone(add_ls(m, i_tow=1001))
        self.assertEqual(m.stats().paired_epochs, 0)

    def test_week_boundary_fractional_tow_normalization(self):
        m = mapper(trusted_leap_sources={2})
        add_gps(m, i_tow=0, f_tow=-100_000, week=2450)
        result = add_ls(m, i_tow=0)
        self.assertEqual(result.gps_week, 2449)
        self.assertEqual(result.source_timestamp_ns, 2450 * GPS_WEEK_NS - 100_000)

    def test_fractional_tow_near_week_end_does_not_carry_early(self):
        m = mapper(trusted_leap_sources={2})
        add_gps(m, i_tow=604_799_999, f_tow=500_000, week=2450)
        result = add_ls(m, i_tow=604_799_999)
        self.assertEqual(result.gps_week, 2450)
        self.assertEqual(result.source_timestamp_ns, 2451 * GPS_WEEK_NS - 500_000)

    def test_negative_fractional_tow_borrows_into_previous_week(self):
        m = mapper(trusted_leap_sources={2})
        add_gps(m, i_tow=0, f_tow=-500_000, week=2450)
        result = add_ls(m, i_tow=0)
        self.assertEqual(result.gps_week, 2449)
        self.assertEqual(result.source_timestamp_ns, 2450 * GPS_WEEK_NS - 500_000)

    def test_invalid_week_with_valid_flag_fails_closed(self):
        m = mapper(trusted_leap_sources={2})
        add_gps(m, week=-1, valid=0x07)
        self.assertIsNone(add_ls(m))
        self.assertEqual(m.stats().invalid_gps_time, 1)

    def test_uint64_source_time_overflow_fails_closed(self):
        m = mapper(trusted_leap_sources={2})
        add_gps(m, i_tow=604_799_999, f_tow=500_000, week=32767)
        self.assertIsNone(add_ls(m, i_tow=604_799_999))
        self.assertEqual(m.stats().invalid_gps_time, 1)

    def test_generation_clock_id_changes_on_week_rollover(self):
        m = mapper(trusted_leap_sources={2})
        add_gps(m, week=2450)
        first = add_ls(m)
        add_gps(m, receive=200, i_tow=1001, week=2451)
        second = add_ls(m, receive=210, i_tow=1001)
        self.assertNotEqual(first.source_clock_id, second.source_clock_id)
        self.assertIn("gps-week:2450", first.source_clock_id)
        self.assertIn("gps-week:2451", second.source_clock_id)

    def test_connection_generation_is_part_of_source_clock_identity(self):
        m = mapper(trusted_leap_sources={2}, connection_generation=7)
        add_gps(m)
        first = add_ls(m)
        self.assertIn(":connection:7:gps-week:2450", first.source_clock_id)
        self.assertEqual(first.connection_generation, 7)

        m.begin_session(SESSION_ID, connection_generation=8)
        add_gps(m, receive=200)
        second = add_ls(m, receive=210)
        self.assertIn(":connection:8:gps-week:2450", second.source_clock_id)
        self.assertNotEqual(first.source_clock_id, second.source_clock_id)

    def test_connection_generation_must_be_non_negative_integer(self):
        m = UbxReceiverTimeMapper()
        for generation in (-1, True, 1.5):
            with self.subTest(generation=generation):
                with self.assertRaisesRegex(EpochPairingError, "connection_generation"):
                    m.begin_session(SESSION_ID, generation)

    def test_old_process_session_callback_is_rejected(self):
        m = mapper()
        stale = gps_epoch(process_session_id="old-process-session", generation=0)
        with self.assertRaisesRegex(EpochPairingError, "stale process_session_id"):
            m.add_timegps(stale)
        self.assertEqual(m.stats().stale_context_rejections, 1)

    def test_old_connection_generation_callback_is_rejected(self):
        m = mapper(connection_generation=2)
        stale = ls_epoch(generation=1)
        with self.assertRaisesRegex(EpochPairingError, "stale process_session_id"):
            m.add_timels(stale)
        self.assertEqual(m.stats().stale_context_rejections, 1)

    def test_new_session_flushes_pending_and_old_frames_cannot_pair(self):
        m = mapper()
        add_gps(m)
        m.begin_session("f9p-usb-session-18", 1)
        old_ls = ls_epoch(process_session_id=SESSION_ID, generation=0)
        with self.assertRaisesRegex(EpochPairingError, "stale process_session_id"):
            m.add_timels(old_ls)
        self.assertEqual(m.stats().paired_epochs, 0)
        new_ls = ls_epoch(process_session_id="f9p-usb-session-18", generation=1)
        self.assertIsNone(m.add_timels(new_ls))
        new_gps = gps_epoch(process_session_id="f9p-usb-session-18", generation=1)
        result = m.add_timegps(new_gps)
        self.assertIsNotNone(result)
        self.assertEqual(result.process_session_id, "f9p-usb-session-18")
        self.assertEqual(result.connection_generation, 1)

    def test_invalid_gps_time_flags_do_not_emit_mapping(self):
        m = mapper(trusted_leap_sources={2})
        add_gps(m, valid=0x04)
        self.assertIsNone(add_ls(m))
        self.assertEqual(m.stats().invalid_gps_time, 1)

    def test_stale_and_capacity_bounds(self):
        m = mapper(max_pending_epochs=1, max_pending_age_ns=20)
        add_gps(m, i_tow=1000, receive=100)
        add_gps(m, i_tow=1001, receive=110)
        self.assertEqual(m.stats().capacity_evictions, 1)
        self.assertEqual(m.expire_stale(131), 1)
        self.assertEqual(m.stats().expired_epochs, 1)

    def test_conflicting_duplicate_is_rejected(self):
        m = mapper()
        add_gps(m)
        with self.assertRaisesRegex(EpochPairingError, "conflicting NAV-TIMEGPS"):
            add_gps(m, receive=101, leap=17)
        self.assertEqual(m.stats().conflicting_duplicates, 1)

    def test_pair_receive_skew_is_enforced(self):
        m = mapper(max_pair_receive_skew_ns=10)
        add_gps(m, receive=100)
        self.assertIsNone(add_ls(m, receive=111))
        self.assertEqual(m.stats().pair_skew_rejections, 1)

    def test_concurrent_timegps_and_timels_callbacks_emit_one_mapping(self):
        m = mapper(trusted_leap_sources={2})
        operations = (
            lambda: add_gps(m, receive=100),
            lambda: add_ls(m, receive=110),
        )
        with ThreadPoolExecutor(max_workers=2) as executor:
            results = list(executor.map(lambda operation: operation(), operations))
        mappings = [result for result in results if result is not None]
        self.assertEqual(len(mappings), 1)
        self.assertTrue(mappings[0].utc_stamp_valid)

    def test_session_reset_clears_pending_and_requires_begin(self):
        m = mapper()
        add_gps(m)
        m.reset()
        with self.assertRaisesRegex(EpochPairingError, "begin_session"):
            m.add_timels(ls_epoch())
        m.begin_session(SESSION_ID, 0)
        self.assertIsNone(m.add_timels(ls_epoch()))
