import unittest
from concurrent.futures import ThreadPoolExecutor

from robodog_gnss_quality import CarrierSolution
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


PROCESS_SESSION_ID = "f9p-process-session-17"
CONNECTION_GENERATION = 3
GPS_WEEK = 2450
GENERATION = f"{PROCESS_SESSION_ID}:connection:{CONNECTION_GENERATION}:gps-week:{GPS_WEEK}"


def pvt(i_tow=1000, receive=100):
    return UbxNavPvtEpoch(PROCESS_SESSION_ID, CONNECTION_GENERATION, i_tow, True, CarrierSolution.FIXED, receive)


def position(i_tow=1000, receive=105):
    return UbxNavHpPosLlhEpoch(
        PROCESS_SESSION_ID, CONNECTION_GENERATION,
        i_tow, -1220000000, 370000000, 15000, 0, 0, 0,
        False, False, False, False, False, False, receive,
    )


def covariance(i_tow=1000, receive=110):
    return UbxNavCovEpoch(PROCESS_SESSION_ID, CONNECTION_GENERATION, i_tow, True, 1.0, 0.1, 0.2, 2.0, 0.3, 3.0, receive)


def status(i_tow=1000, receive=115):
    return UbxNavStatusEpoch(PROCESS_SESSION_ID, CONNECTION_GENERATION, i_tow, True, True, CarrierSolution.FIXED, receive)


def mapping(i_tow=1000, receive=120, source_time=10_000_000, valid=True,
            utc_valid=True, clock_id=GENERATION, session_id=PROCESS_SESSION_ID,
            generation=CONNECTION_GENERATION, gps_week=GPS_WEEK):
    return UbxSourceTimeMapping(
        i_tow_ms=i_tow,
        source_timestamp_ns=source_time,
        source_clock_id=clock_id,
        receive_monotonic_ns=receive,
        valid=valid,
        utc_stamp_valid=utc_valid,
        utc_seconds=1_800_000_000 if utc_valid else None,
        utc_nanoseconds=123 if utc_valid else None,
        validity_reason="valid" if valid else "time source invalid",
        process_session_id=session_id,
        connection_generation=generation,
        gps_week=gps_week,
    )


def adapter(**overrides):
    values = dict(
        max_pending_epochs=4,
        max_pair_receive_skew_ns=50,
        max_pending_age_ns=1000,
    )
    values.update(overrides)
    result = UbxRosEpochAdapter(**values)
    result.begin_generation(
        GENERATION, process_session_id=PROCESS_SESSION_ID,
        connection_generation=CONNECTION_GENERATION, gps_week=GPS_WEEK
    )
    return result


class TestUbxRosEpochAdapter(unittest.TestCase):
    def test_records_wait_until_mapping_arrives(self):
        a = adapter()
        self.assertIsNone(a.add_pvt(pvt()))
        self.assertIsNone(a.add_position(position()))
        self.assertIsNone(a.add_covariance(covariance()))
        self.assertIsNone(a.add_status(status()))
        self.assertEqual(a.pending_count, 1)
        result = a.add_time_mapping(mapping())
        self.assertIsNotNone(result)
        self.assertEqual(result.observation.source_timestamp_ns, 10_000_000)
        self.assertEqual(result.observation.source_clock_id, GENERATION)
        self.assertTrue(result.time_mapping.utc_stamp_valid)
        self.assertEqual(a.pending_count, 0)

    def test_mapping_may_arrive_before_messages(self):
        a = adapter()
        self.assertIsNone(a.add_time_mapping(mapping()))
        self.assertIsNone(a.add_status(status()))
        self.assertIsNone(a.add_covariance(covariance()))
        self.assertIsNone(a.add_position(position()))
        result = a.add_pvt(pvt())
        self.assertIsNotNone(result)
        self.assertEqual(result.observation.latitude_deg, 37.0)

    def test_invalid_time_mapping_retires_epoch(self):
        a = adapter()
        a.add_pvt(pvt())
        self.assertIsNone(a.add_time_mapping(mapping(valid=False)))
        self.assertEqual(a.invalid_mappings, 1)
        self.assertIsNone(a.add_position(position()))
        self.assertEqual(a.pending_count, 0)

    def test_source_clock_change_requires_explicit_generation(self):
        a = adapter()
        next_clock = f"{PROCESS_SESSION_ID}:connection:{CONNECTION_GENERATION}:gps-week:2451"
        with self.assertRaisesRegex(EpochPairingError, "call begin_generation"):
            a.add_time_mapping(mapping(clock_id=next_clock, gps_week=2451))
        a.begin_generation(
            next_clock, process_session_id=PROCESS_SESSION_ID,
            connection_generation=CONNECTION_GENERATION, gps_week=2451
        )
        result = a.add_time_mapping(mapping(clock_id=next_clock, gps_week=2451))
        self.assertIsNone(result)

    def test_conflicting_duplicate_retires_epoch(self):
        a = adapter()
        a.add_pvt(pvt())
        with self.assertRaisesRegex(EpochPairingError, "conflicting duplicate"):
            a.add_pvt(UbxNavPvtEpoch(PROCESS_SESSION_ID, CONNECTION_GENERATION, 1000, False, CarrierSolution.FIXED, 101))
        self.assertEqual(a.pending_count, 0)
        self.assertEqual(a.rejected_conflict, 1)

    def test_capacity_and_stale_expiry_are_bounded(self):
        a = adapter(max_pending_epochs=1, max_pending_age_ns=20)
        a.add_pvt(pvt(1000, 100))
        a.add_pvt(pvt(1001, 110))
        self.assertEqual(a.pending_count, 1)
        self.assertEqual(a.evicted_capacity, 1)
        self.assertEqual(a.expire_stale(131), 1)
        self.assertEqual(a.pending_count, 0)
        self.assertEqual(a.evicted_stale, 1)

    def test_stale_expiry_uses_monotonic_watermark_and_earliest_receive(self):
        a = adapter(max_pending_age_ns=20)
        a.add_time_mapping(mapping(receive=120))
        a.add_pvt(pvt(receive=100))
        self.assertIsNone(a.add_status(status(receive=300)))
        self.assertEqual(a.pending_count, 0)
        self.assertEqual(a.evicted_stale, 1)
        self.assertEqual(a.expire_stale(150), 0)

    def test_utc_stamp_may_remain_unavailable_even_with_valid_gps_mapping(self):
        a = adapter()
        for record, add in (
            (pvt(), a.add_pvt),
            (position(), a.add_position),
            (covariance(), a.add_covariance),
            (status(), a.add_status),
        ):
            add(record)
        result = a.add_time_mapping(mapping(utc_valid=False))
        self.assertIsNotNone(result)
        self.assertFalse(result.time_mapping.utc_stamp_valid)
        self.assertIsNone(result.time_mapping.utc_seconds)

    def test_mapping_conflict_is_rejected(self):
        a = adapter()
        a.add_pvt(pvt())
        a.add_time_mapping(mapping())
        with self.assertRaisesRegex(EpochPairingError, "conflicting receiver-time"):
            a.add_time_mapping(mapping(source_time=10_000_001))
        self.assertEqual(a.pending_count, 0)

    def test_concurrent_topic_callbacks_emit_one_observation(self):
        a = adapter()
        operations = (
            lambda: a.add_pvt(pvt()),
            lambda: a.add_position(position()),
            lambda: a.add_covariance(covariance()),
            lambda: a.add_status(status()),
            lambda: a.add_time_mapping(mapping()),
        )
        with ThreadPoolExecutor(max_workers=5) as executor:
            results = list(executor.map(lambda operation: operation(), operations))
        emitted = [result for result in results if result is not None]
        self.assertEqual(len(emitted), 1)
        self.assertEqual(emitted[0].observation.source_timestamp_ns, 10_000_000)
        self.assertEqual(a.pending_count, 0)

    def test_time_mapping_session_context_must_match_clock_id(self):
        a = adapter()
        bad = mapping(
            session_id="f9p-session",
            generation=3,
            gps_week=2450,
            clock_id=GENERATION,
        )
        with self.assertRaisesRegex(EpochPairingError, "does not match session/generation/week"):
            a.add_time_mapping(bad)

    def test_time_mapping_context_is_preserved_in_result(self):
        source_id = "f9p-session:connection:3:gps-week:2450"
        a = adapter()
        a.begin_generation(
            source_id, process_session_id="f9p-session",
            connection_generation=3, gps_week=2450
        )
        for record, add in (
            (pvt(), a.add_pvt),
            (position(), a.add_position),
            (covariance(), a.add_covariance),
            (status(), a.add_status),
        ):
            add(record)
        result = a.add_time_mapping(mapping(
            clock_id=source_id,
            session_id="f9p-session",
            generation=3,
            gps_week=2450,
        ))
        self.assertIsNotNone(result)
        self.assertEqual(result.time_mapping.process_session_id, "f9p-session")
        self.assertEqual(result.time_mapping.connection_generation, 3)

    def test_old_generation_ros_callback_is_rejected_after_reconnect(self):
        a = adapter()
        old_record = pvt()
        new_generation = CONNECTION_GENERATION + 1
        new_clock = f"{PROCESS_SESSION_ID}:connection:{new_generation}:gps-week:{GPS_WEEK}"
        a.begin_generation(
            new_clock, process_session_id=PROCESS_SESSION_ID,
            connection_generation=new_generation, gps_week=GPS_WEEK
        )
        with self.assertRaisesRegex(EpochPairingError, "stale process_session_id/connection_generation"):
            a.add_pvt(old_record)
        new_record = UbxNavPvtEpoch(
            PROCESS_SESSION_ID, new_generation, 1000, True,
            CarrierSolution.FIXED, 101
        )
        self.assertIsNone(a.add_pvt(new_record))

    def test_reset_requires_explicit_new_generation(self):
        a = adapter()
        a.reset()
        with self.assertRaisesRegex(EpochPairingError, "begin_generation"):
            a.add_pvt(pvt())
        a.begin_generation(
            GENERATION, process_session_id=PROCESS_SESSION_ID,
            connection_generation=CONNECTION_GENERATION, gps_week=GPS_WEEK
        )
        self.assertIsNone(a.add_pvt(pvt()))


if __name__ == "__main__":
    unittest.main()
