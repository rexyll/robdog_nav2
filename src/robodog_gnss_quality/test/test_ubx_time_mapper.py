import unittest
from concurrent.futures import ThreadPoolExecutor

from robodog_gnss_quality.ubx_time_mapper import (
    GPS_EPOCH_UNIX_NS,
    GPS_WEEK_NS,
    UbxReceiverTimeMapper,
    decode_nav_timegps,
    decode_nav_timels,
)
from robodog_gnss_quality.ubx_nav_pvt import EpochPairingError


def timegps(i_tow=1000, f_tow=250_000, week=2450, leap=18, valid=0x07, tacc=50):
    return (
        i_tow.to_bytes(4, "little")
        + f_tow.to_bytes(4, "little", signed=True)
        + week.to_bytes(2, "little", signed=True)
        + leap.to_bytes(1, "little", signed=True)
        + bytes([valid])
        + tacc.to_bytes(4, "little")
    )


def timels(i_tow=1000, source=2, leap=18, valid=0x03, version=0, time_to_event=86_400):
    payload = bytearray(24)
    payload[0:4] = i_tow.to_bytes(4, "little")
    payload[4] = version
    payload[8] = source
    payload[9:10] = leap.to_bytes(1, "little", signed=True)
    payload[10] = 2
    payload[11:12] = (1).to_bytes(1, "little", signed=True)
    payload[12:16] = time_to_event.to_bytes(4, "little", signed=True)
    payload[16:18] = (2451).to_bytes(2, "little")
    payload[18:20] = (7).to_bytes(2, "little")
    payload[23] = valid
    return bytes(payload)


def mapper(**kwargs):
    result = UbxReceiverTimeMapper(**kwargs)
    result.begin_session("f9p-usb-session-17")
    return result


class TestUbxTimeMapper(unittest.TestCase):
    def test_decode_timegps_fields_and_validity(self):
        decoded = decode_nav_timegps(timegps())
        self.assertEqual(decoded.i_tow_ms, 1000)
        self.assertEqual(decoded.f_tow_ns, 250_000)
        self.assertEqual(decoded.week, 2450)
        self.assertTrue(decoded.tow_valid)
        self.assertTrue(decoded.week_valid)
        self.assertTrue(decoded.leap_seconds_valid)
        self.assertEqual(decoded.time_accuracy_ns, 50)

    def test_decode_timels_fields_and_validity(self):
        decoded = decode_nav_timels(timels())
        self.assertEqual(decoded.source_of_current_leap, 2)
        self.assertEqual(decoded.current_leap_seconds, 18)
        self.assertTrue(decoded.current_leap_valid)
        self.assertTrue(decoded.leap_event_valid)
        self.assertEqual(decoded.time_to_leap_event_seconds, 86_400)

    def test_strict_payload_lengths_and_versions(self):
        with self.assertRaises(EpochPairingError):
            decode_nav_timegps(b"short")
        with self.assertRaises(EpochPairingError):
            decode_nav_timels(bytes(23))
        with self.assertRaises(EpochPairingError):
            decode_nav_timels(timels(version=1))
        with self.assertRaises(EpochPairingError):
            decode_nav_timegps(timegps(i_tow=604_800_000))
        with self.assertRaises(EpochPairingError):
            decode_nav_timegps(timegps(f_tow=500_001))
        invalid_timels = bytearray(timels())
        invalid_timels[11] = 2
        with self.assertRaisesRegex(EpochPairingError, "lsChange"):
            decode_nav_timels(bytes(invalid_timels))

    def test_pair_in_either_order_and_convert_utc_only_with_trusted_source(self):
        m = mapper(trusted_leap_sources={2})
        self.assertIsNone(m.add_timegps(timegps(), 1_000))
        result = m.add_timels(timels(), 1_100)
        self.assertIsNotNone(result)
        expected_gps_ns = 2450 * GPS_WEEK_NS + 1_000_250_000
        self.assertEqual(result.source_timestamp_ns, expected_gps_ns)
        self.assertTrue(result.valid)
        self.assertTrue(result.utc_stamp_valid)
        self.assertEqual(result.gps_week, 2450)
        expected_utc_ns = GPS_EPOCH_UNIX_NS + expected_gps_ns - 18_000_000_000
        self.assertEqual(result.utc_seconds, expected_utc_ns // 1_000_000_000)
        self.assertEqual(result.utc_nanoseconds, expected_utc_ns % 1_000_000_000)

        m2 = mapper(trusted_leap_sources={2})
        self.assertIsNone(m2.add_timels(timels(), 1_100))
        result2 = m2.add_timegps(timegps(), 1_000)
        self.assertEqual(result2, result)

    def test_untrusted_or_invalid_leap_source_keeps_gps_time_but_not_utc(self):
        m = mapper()  # No leap source is trusted by default.
        self.assertIsNone(m.add_timegps(timegps(), 100))
        result = m.add_timels(timels(source=2), 110)
        self.assertTrue(result.valid)
        self.assertFalse(result.utc_stamp_valid)
        self.assertIsNone(result.utc_seconds)
        self.assertIn("not trusted", result.validity_reason)

        m2 = mapper(trusted_leap_sources={2})
        m2.add_timegps(timegps(), 100)
        mismatch = m2.add_timels(timels(leap=17), 110)
        self.assertTrue(mismatch.valid)
        self.assertFalse(mismatch.utc_stamp_valid)
        self.assertIn("differs", mismatch.validity_reason)

    def test_beidou_leap_source_is_adjusted_to_gps_utc_before_cross_check(self):
        m = mapper(trusted_leap_sources={4})
        m.add_timegps(timegps(leap=18), 100)
        result = m.add_timels(timels(source=4, leap=4), 110)
        self.assertTrue(result.utc_stamp_valid)
        self.assertEqual(result.leap_seconds, 18)
        self.assertEqual(result.reported_leap_seconds, 4)

    def test_current_leap_validity_is_independent_of_event_schedule_and_guard(self):
        m = mapper(trusted_leap_sources={2})
        m.add_timegps(timegps(), 100)
        no_event_time = m.add_timels(timels(valid=0x01, change=2), 110)
        self.assertTrue(no_event_time.valid)
        self.assertTrue(no_event_time.utc_stamp_valid)
        self.assertFalse(no_event_time.leap_event_valid)
        self.assertIsNone(no_event_time.time_to_leap_event_seconds)
        self.assertTrue(no_event_time.utc_stamp_valid)

        for offset in (-1, 0, 1):
            guarded = mapper(trusted_leap_sources={2})
            guarded.add_timegps(timegps(), 100)
            result = guarded.add_timels(timels(time_to_event=offset), 110)
            self.assertTrue(result.valid)
            self.assertFalse(result.utc_stamp_valid)
            self.assertIn("transition guard window", result.validity_reason)

        outside = mapper(trusted_leap_sources={2})
        outside.add_timegps(timegps(), 100)
        result = outside.add_timels(timels(time_to_event=2), 110)
        self.assertTrue(result.utc_stamp_valid)

    def test_firmware_default_leap_source_cannot_be_allowlisted(self):
        with self.assertRaisesRegex(ValueError, "firmware default 0 is never trusted"):
            UbxReceiverTimeMapper(trusted_leap_sources={0})

    def test_utc_stamp_is_withheld_near_valid_leap_event(self):
        m = mapper(trusted_leap_sources={2})
        m.add_timegps(timegps(), 100)
        result = m.add_timels(timels(time_to_event=1), 110)
        self.assertTrue(result.valid)
        self.assertFalse(result.utc_stamp_valid)
        self.assertTrue(result.leap_event_valid)
        self.assertEqual(result.time_to_leap_event_seconds, 1)
        self.assertIn("transition guard window", result.validity_reason)

    def test_negative_or_unrepresentable_gps_utc_leap_offset_blocks_utc(self):
        m = mapper(trusted_leap_sources={2})
        m.add_timegps(timegps(leap=-1), 100)
        result = m.add_timels(timels(leap=-1), 110)
        self.assertTrue(result.valid)
        self.assertFalse(result.utc_stamp_valid)
        self.assertIn("outside 0..127", result.validity_reason)

    def test_invalid_future_event_fields_do_not_invalidate_current_leap(self):
        payload = bytearray(timels(valid=0x01))
        payload[11] = 2  # invalid lsChange, but validTimeToLsEvent is clear
        decoded = decode_nav_timels(bytes(payload))
        self.assertTrue(decoded.current_leap_valid)
        self.assertFalse(decoded.leap_event_valid)
        m = mapper(trusted_leap_sources={2})
        m.add_timegps(timegps(), 100)
        result = m.add_timels(bytes(payload), 110)
        self.assertTrue(result.utc_stamp_valid)
        self.assertIsNone(result.time_to_leap_event_seconds)

    def test_timegps_leap_flag_and_timels_validity_are_both_required(self):
        m = mapper(trusted_leap_sources={2})
        m.add_timegps(timegps(valid=0x03), 100)
        result = m.add_timels(timels(), 110)
        self.assertTrue(result.valid)
        self.assertFalse(result.utc_stamp_valid)

        m2 = mapper(trusted_leap_sources={2})
        m2.add_timegps(timegps(), 100)
        result2 = m2.add_timels(timels(valid=0x02), 110)
        self.assertFalse(result2.utc_stamp_valid)

    def test_timegps_and_timels_must_match_exact_itow(self):
        m = mapper(trusted_leap_sources={2})
        self.assertIsNone(m.add_timegps(timegps(i_tow=1000), 100))
        self.assertIsNone(m.add_timels(timels(i_tow=1001), 110))
        self.assertEqual(m.stats().paired_epochs, 0)

    def test_week_boundary_fractional_tow_normalization(self):
        m = mapper(trusted_leap_sources={2})
        m.add_timegps(timegps(i_tow=0, f_tow=-100_000, week=2450), 100)
        result = m.add_timels(timels(i_tow=0), 110)
        self.assertEqual(result.gps_week, 2449)
        self.assertEqual(result.source_timestamp_ns, 2450 * GPS_WEEK_NS - 100_000)

    def test_generation_clock_id_changes_on_week_rollover(self):
        m = mapper(trusted_leap_sources={2})
        m.add_timegps(timegps(week=2450), 100)
        first = m.add_timels(timels(), 110)
        m.add_timegps(timegps(i_tow=1001, week=2451), 200)
        second = m.add_timels(timels(i_tow=1001), 210)
        self.assertNotEqual(first.source_clock_id, second.source_clock_id)
        self.assertIn("gps-week:2450", first.source_clock_id)
        self.assertIn("gps-week:2451", second.source_clock_id)

    def test_invalid_gps_time_flags_do_not_emit_mapping(self):
        m = mapper(trusted_leap_sources={2})
        m.add_timegps(timegps(valid=0x04), 100)
        self.assertIsNone(m.add_timels(timels(), 110))
        self.assertEqual(m.stats().invalid_gps_time, 1)

    def test_stale_and_capacity_bounds(self):
        m = mapper(max_pending_epochs=1, max_pending_age_ns=20)
        m.add_timegps(timegps(i_tow=1000), 100)
        m.add_timegps(timegps(i_tow=1001), 110)
        self.assertEqual(m.stats().capacity_evictions, 1)
        self.assertEqual(m.expire_stale(131), 1)
        self.assertEqual(m.stats().expired_epochs, 1)

    def test_conflicting_duplicate_is_rejected(self):
        m = mapper()
        m.add_timegps(timegps(), 100)
        with self.assertRaisesRegex(EpochPairingError, "conflicting NAV-TIMEGPS"):
            m.add_timegps(timegps(leap=17), 101)
        self.assertEqual(m.stats().conflicting_duplicates, 1)

    def test_concurrent_timegps_and_timels_callbacks_emit_one_mapping(self):
        m = mapper(trusted_leap_sources={2})
        with ThreadPoolExecutor(max_workers=2) as executor:
            results = list(executor.map(
                lambda item: item[0](*item[1]),
                (
                    (m.add_timegps, (timegps(), 100)),
                    (m.add_timels, (timels(), 110)),
                ),
            ))
        mappings = [result for result in results if result is not None]
        self.assertEqual(len(mappings), 1)
        self.assertTrue(mappings[0].utc_stamp_valid)

    def test_session_reset_clears_pending_and_requires_begin(self):
        m = mapper()
        m.add_timegps(timegps(), 100)
        m.reset()
        with self.assertRaisesRegex(EpochPairingError, "begin_session"):
            m.add_timels(timels(), 110)


if __name__ == "__main__":
    unittest.main()
