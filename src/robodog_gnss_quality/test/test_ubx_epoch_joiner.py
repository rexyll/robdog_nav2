import unittest
from dataclasses import replace

from robodog_gnss_quality import CarrierSolution
from robodog_gnss_quality.ubx_epoch_joiner import UbxFourMessageEpochJoiner
from robodog_gnss_quality.ubx_nav_pvt import (
    EpochPairingError,
    UbxNavCovEpoch,
    UbxNavHpPosLlhEpoch,
    UbxNavPvtEpoch,
    UbxNavStatusEpoch,
)


PROCESS_SESSION_ID = "joiner-test-session"
CONNECTION_GENERATION = 0
GPS_WEEK = 2450
GENERATION = f"{PROCESS_SESSION_ID}:connection:{CONNECTION_GENERATION}:gps-week:2450"


def make_pvt(i_tow=1000, receive=100, process_session_id=PROCESS_SESSION_ID, connection_generation=CONNECTION_GENERATION, gps_week=GPS_WEEK):
    return UbxNavPvtEpoch(
        process_session_id=process_session_id,
        connection_generation=connection_generation,
        gps_week=gps_week,
        i_tow_ms=i_tow,
        gnss_fix_ok=True,
        carrier_solution=CarrierSolution.FIXED,
        receive_monotonic_ns=receive,
    )


def make_position(i_tow=1000, receive=110):
    return UbxNavHpPosLlhEpoch(
        process_session_id=PROCESS_SESSION_ID,
        connection_generation=CONNECTION_GENERATION,
        gps_week=GPS_WEEK,
        i_tow_ms=i_tow,
        lon=-1_220_000_000,
        lat=370_000_000,
        height_mm=15_000,
        lon_hp=0,
        lat_hp=0,
        height_hp=0,
        invalid_lon=False,
        invalid_lat=False,
        invalid_height=False,
        invalid_lon_hp=False,
        invalid_lat_hp=False,
        invalid_height_hp=False,
        receive_monotonic_ns=receive,
    )


def make_covariance(i_tow=1000, receive=120):
    return UbxNavCovEpoch(
        process_session_id=PROCESS_SESSION_ID,
        connection_generation=CONNECTION_GENERATION,
        gps_week=GPS_WEEK,
        i_tow_ms=i_tow,
        pos_cor_valid=True,
        pos_cov_nn=1.0,
        pos_cov_ne=0.1,
        pos_cov_nd=0.2,
        pos_cov_ee=2.0,
        pos_cov_ed=0.3,
        pos_cov_dd=3.0,
        receive_monotonic_ns=receive,
    )


def make_status(i_tow=1000, receive=130):
    return UbxNavStatusEpoch(
        process_session_id=PROCESS_SESSION_ID,
        connection_generation=CONNECTION_GENERATION,
        gps_week=GPS_WEEK,
        i_tow_ms=i_tow,
        gnss_fix_ok=True,
        carrier_solution_valid=True,
        carrier_solution=CarrierSolution.FIXED,
        receive_monotonic_ns=receive,
    )


def make_joiner(**overrides):
    values = dict(
        max_pending_epochs=8,
        max_pair_receive_skew_ns=50,
        max_pending_age_ns=1000,
    )
    values.update(overrides)
    return UbxFourMessageEpochJoiner(**values)


def add_epoch(joiner, *, i_tow=1000, source_time=1_000_000, order=None):
    records = {
        "pvt": make_pvt(i_tow, 100),
        "position": make_position(i_tow, 110),
        "covariance": make_covariance(i_tow, 120),
        "status": make_status(i_tow, 130),
    }
    order = order or ("pvt", "position", "covariance", "status")
    result = None
    for kind in order:
        result = getattr(joiner, "add_" + kind)(
            records[kind],
            source_clock_id=GENERATION,
            source_timestamp_ns=source_time,
        )
    return result


class TestUbxFourMessageEpochJoiner(unittest.TestCase):
    def test_out_of_order_topic_delivery_joins_exact_epoch(self):
        joiner = make_joiner()
        observation = add_epoch(
            joiner,
            order=("status", "covariance", "pvt", "position"),
        )
        self.assertIsNotNone(observation)
        self.assertEqual(observation.source_timestamp_ns, 1_000_000)
        self.assertEqual(observation.source_clock_id, GENERATION)
        self.assertEqual(observation.carrier_solution, CarrierSolution.FIXED)
        self.assertEqual(joiner.pending_count, 0)

    def test_old_receiver_generation_record_is_rejected_by_source_clock_id(self):
        joiner = make_joiner()
        stale = make_pvt().__class__(
            process_session_id="old-process-session",
            connection_generation=CONNECTION_GENERATION,
            gps_week=GPS_WEEK,
            i_tow_ms=1000,
            gnss_fix_ok=True,
            carrier_solution=CarrierSolution.FIXED,
            receive_monotonic_ns=100,
        )
        with self.assertRaisesRegex(EpochPairingError, "stale process_session_id"):
            joiner.add_pvt(
                stale, source_clock_id=GENERATION, source_timestamp_ns=1_000_000
            )

    def test_old_gps_week_record_is_rejected_even_with_same_connection_generation(self):
        joiner = make_joiner()
        stale = make_pvt(gps_week=GPS_WEEK - 1)
        with self.assertRaisesRegex(EpochPairingError, "gps_week"):
            joiner.add_pvt(
                stale, source_clock_id=GENERATION, source_timestamp_ns=1_000_000
            )


    def test_all_four_message_types_reject_stale_gps_week(self):
        joiner = make_joiner()
        records = (
            ("pvt", make_pvt(gps_week=GPS_WEEK - 1)),
            ("position", replace(make_position(), gps_week=GPS_WEEK - 1)),
            ("covariance", replace(make_covariance(), gps_week=GPS_WEEK - 1)),
            ("status", replace(make_status(), gps_week=GPS_WEEK - 1)),
        )
        for kind, record in records:
            with self.subTest(kind=kind):
                with self.assertRaisesRegex(EpochPairingError, "gps_week"):
                    getattr(joiner, "add_" + kind)(
                        record, source_clock_id=GENERATION,
                        source_timestamp_ns=1_000_000,
                    )

    def test_different_itow_samples_never_cross_join(self):
        joiner = make_joiner()
        joiner.add_pvt(
            make_pvt(1000), source_clock_id=GENERATION, source_timestamp_ns=1_000_000
        )
        joiner.add_status(
            make_status(1001), source_clock_id=GENERATION, source_timestamp_ns=2_000_000
        )
        joiner.add_position(
            make_position(1000), source_clock_id=GENERATION, source_timestamp_ns=1_000_000
        )
        joiner.add_covariance(
            make_covariance(1001), source_clock_id=GENERATION, source_timestamp_ns=2_000_000
        )
        self.assertEqual(joiner.pending_count, 2)

        joiner.add_covariance(
            make_covariance(1000), source_clock_id=GENERATION, source_timestamp_ns=1_000_000
        )
        result_a = joiner.add_status(
            make_status(1000), source_clock_id=GENERATION, source_timestamp_ns=1_000_000
        )
        self.assertIsNotNone(result_a)
        self.assertEqual(result_a.source_timestamp_ns, 1_000_000)

        joiner.add_pvt(
            make_pvt(1001), source_clock_id=GENERATION, source_timestamp_ns=2_000_000
        )
        result_b = joiner.add_position(
            make_position(1001), source_clock_id=GENERATION, source_timestamp_ns=2_000_000
        )
        self.assertIsNotNone(result_b)
        self.assertEqual(result_b.source_timestamp_ns, 2_000_000)
        self.assertEqual(joiner.pending_count, 0)

    def test_pending_cache_is_capacity_bounded(self):
        joiner = make_joiner(max_pending_epochs=2)
        for i_tow in (1000, 1001, 1002):
            joiner.add_pvt(
                make_pvt(i_tow),
                source_clock_id=GENERATION,
                source_timestamp_ns=1_000_000 + i_tow,
            )
        self.assertEqual(joiner.pending_count, 2)
        self.assertEqual(joiner.evicted_capacity, 1)

    def test_stale_incomplete_epoch_is_evicted(self):
        joiner = make_joiner(max_pending_age_ns=40)
        joiner.add_pvt(
            make_pvt(receive=100),
            source_clock_id=GENERATION,
            source_timestamp_ns=1_000_000,
        )
        self.assertEqual(joiner.expire_stale(140), 0)
        self.assertEqual(joiner.expire_stale(141), 1)
        self.assertEqual(joiner.pending_count, 0)
        self.assertEqual(joiner.evicted_stale, 1)

    def test_identical_duplicate_ignores_new_callback_receive_time(self):
        joiner = make_joiner()
        joiner.add_pvt(
            make_pvt(receive=100),
            source_clock_id=GENERATION,
            source_timestamp_ns=1_000_000,
        )
        result = joiner.add_pvt(
            make_pvt(receive=101),
            source_clock_id=GENERATION,
            source_timestamp_ns=1_000_000,
        )
        self.assertIsNone(result)
        self.assertEqual(joiner.pending_count, 1)
        self.assertEqual(joiner.rejected_conflict, 0)

    def test_conflicting_duplicate_drops_incomplete_epoch(self):
        joiner = make_joiner()
        joiner.add_pvt(
            make_pvt(receive=100),
            source_clock_id=GENERATION,
            source_timestamp_ns=1_000_000,
        )
        with self.assertRaisesRegex(EpochPairingError, "conflicting duplicate"):
            joiner.add_pvt(
                make_pvt(receive=101).__class__(
                    process_session_id=PROCESS_SESSION_ID,
                    connection_generation=CONNECTION_GENERATION,
                    i_tow_ms=1000,
                    gnss_fix_ok=False,
                    carrier_solution=CarrierSolution.FIXED,
                    receive_monotonic_ns=101,
                ),
                source_clock_id=GENERATION,
                source_timestamp_ns=1_000_000,
            )
        self.assertEqual(joiner.pending_count, 0)
        self.assertEqual(joiner.rejected_conflict, 1)

    def test_source_timestamp_mapping_must_match_within_one_itow(self):
        joiner = make_joiner()
        joiner.add_pvt(
            make_pvt(), source_clock_id=GENERATION, source_timestamp_ns=1_000_000
        )
        with self.assertRaisesRegex(EpochPairingError, "mapping changed"):
            joiner.add_position(
                make_position(),
                source_clock_id=GENERATION,
                source_timestamp_ns=1_000_001,
            )
        self.assertEqual(joiner.pending_count, 0)

    def test_receiver_generation_change_requires_explicit_reset(self):
        joiner = make_joiner()
        joiner.begin_generation(GENERATION)
        joiner.add_pvt(
            make_pvt(), source_clock_id=GENERATION, source_timestamp_ns=1_000_000
        )
        with self.assertRaisesRegex(EpochPairingError, "begin_generation"):
            joiner.add_position(
                make_position(),
                source_clock_id=f"{PROCESS_SESSION_ID}:connection:{CONNECTION_GENERATION}:gps-week:2451",
                source_timestamp_ns=2_000_000,
            )
        self.assertEqual(joiner.pending_count, 1)

        joiner.begin_generation(f"{PROCESS_SESSION_ID}:connection:{CONNECTION_GENERATION}:gps-week:2451")
        self.assertEqual(joiner.pending_count, 0)
        self.assertIsNone(joiner.add_pvt(
            make_pvt(), source_clock_id=f"{PROCESS_SESSION_ID}:connection:{CONNECTION_GENERATION}:gps-week:2451",
            source_timestamp_ns=2_000_000,
        ))

    def test_duplicate_completed_epoch_does_not_emit_twice(self):
        joiner = make_joiner()
        observation = add_epoch(joiner)
        self.assertIsNotNone(observation)
        duplicate = joiner.add_pvt(
            make_pvt(), source_clock_id=GENERATION, source_timestamp_ns=1_000_000
        )
        self.assertIsNone(duplicate)
        self.assertEqual(joiner.pending_count, 0)

    def test_completed_epochs_must_be_monotonic(self):
        joiner = make_joiner()
        self.assertIsNotNone(add_epoch(joiner, i_tow=1000, source_time=2_000_000))
        with self.assertRaisesRegex(EpochPairingError, "not newer"):
            add_epoch(joiner, i_tow=999, source_time=1_000_000)
        self.assertEqual(joiner.rejected_out_of_order, 1)

    def test_receive_skew_failure_discards_completed_candidate(self):
        joiner = make_joiner(max_pair_receive_skew_ns=10)
        joiner.add_pvt(
            make_pvt(receive=100), source_clock_id=GENERATION, source_timestamp_ns=1_000_000
        )
        joiner.add_position(
            make_position(receive=105), source_clock_id=GENERATION, source_timestamp_ns=1_000_000
        )
        joiner.add_covariance(
            make_covariance(receive=110), source_clock_id=GENERATION, source_timestamp_ns=1_000_000
        )
        with self.assertRaisesRegex(EpochPairingError, "receive times"):
            joiner.add_status(
                make_status(receive=111), source_clock_id=GENERATION, source_timestamp_ns=1_000_000
            )
        self.assertEqual(joiner.pending_count, 0)

    def test_constructor_rejects_non_positive_bounds(self):
        for kwargs in (
            {"max_pending_epochs": 0},
            {"max_pair_receive_skew_ns": 0},
            {"max_pending_age_ns": 0},
            {"max_pending_epochs": True},
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    make_joiner(**kwargs)

    def test_capacity_evicted_epoch_cannot_be_resurrected(self):
        joiner = make_joiner(max_pending_epochs=1)
        joiner.add_pvt(
            make_pvt(1000, 100),
            source_clock_id=GENERATION,
            source_timestamp_ns=1_000_000,
        )
        joiner.add_pvt(
            make_pvt(1001, 110),
            source_clock_id=GENERATION,
            source_timestamp_ns=2_000_000,
        )
        self.assertEqual(joiner.pending_count, 1)
        self.assertEqual(joiner.evicted_capacity, 1)

        self.assertIsNone(joiner.add_position(
            make_position(1000, 111),
            source_clock_id=GENERATION,
            source_timestamp_ns=1_000_000,
        ))
        self.assertIsNone(joiner.add_covariance(
            make_covariance(1000, 112),
            source_clock_id=GENERATION,
            source_timestamp_ns=1_000_000,
        ))
        self.assertIsNone(joiner.add_status(
            make_status(1000, 113),
            source_clock_id=GENERATION,
            source_timestamp_ns=1_000_000,
        ))
        self.assertEqual(joiner.pending_count, 1)
        self.assertEqual(joiner.retired_count, 1)

    def test_stale_epoch_cannot_be_resurrected_after_expiry(self):
        joiner = make_joiner(max_pending_age_ns=40)
        joiner.add_pvt(
            make_pvt(1000, 100),
            source_clock_id=GENERATION,
            source_timestamp_ns=1_000_000,
        )
        self.assertEqual(joiner.expire_stale(141), 1)
        self.assertIsNone(joiner.add_position(
            make_position(1000, 142),
            source_clock_id=GENERATION,
            source_timestamp_ns=1_000_000,
        ))
        self.assertIsNone(joiner.add_covariance(
            make_covariance(1000, 143),
            source_clock_id=GENERATION,
            source_timestamp_ns=1_000_000,
        ))
        self.assertIsNone(joiner.add_status(
            make_status(1000, 144),
            source_clock_id=GENERATION,
            source_timestamp_ns=1_000_000,
        ))
        self.assertEqual(joiner.pending_count, 0)
        self.assertEqual(joiner.evicted_stale, 1)

    def test_explicit_expiry_advances_clock_and_rejects_late_callback(self):
        joiner = make_joiner(
            max_pair_receive_skew_ns=10,
            max_pending_age_ns=1000,
        )
        joiner.add_pvt(
            make_pvt(1000, 100),
            source_clock_id=GENERATION,
            source_timestamp_ns=1_000_000,
        )
        self.assertEqual(joiner.expire_stale(200), 0)
        with self.assertRaisesRegex(EpochPairingError, "outside the monotonic receive-time window"):
            joiner.add_status(
                make_status(1000, 189),
                source_clock_id=GENERATION,
                source_timestamp_ns=1_000_000,
            )
        self.assertEqual(joiner.pending_count, 0)
        self.assertEqual(joiner.retired_count, 1)

    def test_i_tow_outside_gps_week_is_rejected(self):
        joiner = make_joiner()
        with self.assertRaisesRegex(EpochPairingError, "GPS week"):
            joiner.add_pvt(
                make_pvt(604_800_000, 100),
                source_clock_id=GENERATION,
                source_timestamp_ns=1_000_000,
            )

    def test_reset_requires_generation_to_be_reestablished(self):
        joiner = make_joiner()
        joiner.begin_generation(GENERATION)
        joiner.reset()
        self.assertIsNone(joiner.source_clock_id)
        self.assertEqual(joiner.pending_count, 0)
        with self.assertRaisesRegex(EpochPairingError, "begin_generation"):
            joiner.add_pvt(
                make_pvt(),
                source_clock_id=GENERATION,
                source_timestamp_ns=1_000_000,
            )
        joiner.begin_generation(GENERATION)
        self.assertIsNone(joiner.add_pvt(
            make_pvt(),
            source_clock_id=GENERATION,
            source_timestamp_ns=1_000_000,
        ))


    def test_reset_preserves_lifetime_counters_and_rejects_old_generation(self):
        joiner = make_joiner(max_pending_epochs=1)
        joiner.begin_generation(GENERATION)
        joiner.add_pvt(
            make_pvt(1000, 100),
            source_clock_id=GENERATION,
            source_timestamp_ns=1_000_000,
        )
        joiner.add_pvt(
            make_pvt(1001, 110),
            source_clock_id=GENERATION,
            source_timestamp_ns=2_000_000,
        )
        self.assertEqual(joiner.evicted_capacity, 1)
        self.assertEqual(joiner.pending_count, 1)

        joiner.reset()
        self.assertIsNone(joiner.source_clock_id)
        self.assertEqual(joiner.pending_count, 0)
        self.assertEqual(joiner.retired_count, 0)
        self.assertEqual(joiner.evicted_capacity, 1)

        new_generation = f"{PROCESS_SESSION_ID}:connection:1:gps-week:2451"
        joiner.begin_generation(new_generation)
        with self.assertRaisesRegex(EpochPairingError, "source_clock_id changed"):
            joiner.add_pvt(
                make_pvt(1002, 120),
                source_clock_id=GENERATION,
                source_timestamp_ns=3_000_000,
            )
        self.assertEqual(joiner.source_clock_id, new_generation)
        self.assertEqual(joiner.pending_count, 0)
        self.assertEqual(joiner.evicted_capacity, 1)

        # The current generation can accept a fresh sample after the stale
        # callback was rejected, without inheriting old epoch cache state.
        self.assertIsNone(joiner.add_pvt(
            make_pvt(1002, 121, connection_generation=1, gps_week=2451),
            source_clock_id=new_generation,
            source_timestamp_ns=4_000_000,
        ))
        self.assertEqual(joiner.pending_count, 1)


if __name__ == "__main__":
    unittest.main()
