import pytest

from robodog_gnss_quality import (
    CarrierSolution, CovarianceType, GnssObservation, GateConfig,
    GnssObservationIdentity, GnssObservationQualityJoiner,
    QualityDecision, RejectionReason,
)
from robodog_gnss_quality.ubx_nav_pvt import EpochPairingError


def identity(i_tow=100, generation=2, week=2450, receive_ns=1000):
    session = "join-test"
    return GnssObservationIdentity(
        process_session_id=session,
        connection_generation=generation,
        gps_week=week,
        i_tow_ms=i_tow,
        source_timestamp_ns=1_000_000_000 + i_tow,
        source_clock_id=f"{session}:connection:{generation}:gps-week:{week}",
        receive_monotonic_ns=receive_ns,
    )


def observation():
    return GnssObservation(
        source_timestamp_ns=1_000_000_100,
        source_clock_id="join-test:connection:2:gps-week:2450",
        receive_monotonic_ns=1000,
        latitude_deg=37.0,
        longitude_deg=-122.0,
        altitude_m=10.0,
        navsat_status=0,
        gnss_fix_ok=True,
        carrier_solution=CarrierSolution.FIXED,
        position_covariance=(0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.04),
        covariance_type=CovarianceType.KNOWN,
    )


def decision(qualified=True):
    return QualityDecision(
        qualified=qualified,
        reason=RejectionReason.ACCEPTED if qualified else RejectionReason.RTK_NOT_FIXED,
        consecutive_fixed_samples=3 if qualified else 0,
        horizontal_std_m=0.1 if qualified else None,
        vertical_std_m=0.2 if qualified else None,
    )


def joiner(**kwargs):
    values = {"max_pending_epochs": 4, "max_pending_age_ns": 100}
    values.update(kwargs)
    return GnssObservationQualityJoiner(**values)


def test_observation_then_matching_qualified_decision_emits_once():
    j = joiner()
    key = identity()
    assert j.add_observation(key, observation(), now_monotonic_ns=1000) is None
    result = j.add_decision(key, decision(), now_monotonic_ns=1010)
    assert result is not None
    assert result.identity == key
    assert result.observation == observation()
    assert result.quality_decision.qualified is True
    assert j.add_observation(key, observation(), now_monotonic_ns=1020) is None
    assert j.stats().emitted == 1
    assert j.stats().retired_drops == 1


def test_qualified_decision_can_arrive_before_observation():
    j = joiner()
    key = identity()
    assert j.add_decision(key, decision(), now_monotonic_ns=1000) is None
    result = j.add_observation(key, observation(), now_monotonic_ns=1010)
    assert result is not None
    assert result.quality_decision.qualified is True


def test_unqualified_decision_retires_epoch_without_emission():
    j = joiner()
    key = identity()
    assert j.add_decision(key, decision(False), now_monotonic_ns=1000) is None
    assert j.add_observation(key, observation(), now_monotonic_ns=1010) is None
    assert j.stats().rejected_unqualified == 1
    assert j.stats().emitted == 0


def test_different_generation_week_or_source_time_never_joins():
    j = joiner()
    obs_key = identity()
    wrong_key = identity(generation=3)
    j.add_observation(obs_key, observation(), now_monotonic_ns=1000)
    assert j.add_decision(wrong_key, decision(), now_monotonic_ns=1010) is None
    assert j.stats().pending_observations == 1
    assert j.stats().pending_decisions == 1


def test_conflicting_duplicate_observation_retires_key():
    j = joiner()
    key = identity()
    j.add_observation(key, observation(), now_monotonic_ns=1000)
    changed = GnssObservation(**{**observation().__dict__, "latitude_deg": 38.0})
    assert j.add_observation(key, changed, now_monotonic_ns=1010) is None
    assert j.add_decision(key, decision(), now_monotonic_ns=1020) is None
    assert j.stats().conflicting_observations == 1
    assert j.stats().emitted == 0


def test_identical_duplicate_records_are_ignored():
    j = joiner()
    key = identity()
    j.add_decision(key, decision(), now_monotonic_ns=1000)
    j.add_decision(key, decision(), now_monotonic_ns=1010)
    j.add_observation(key, observation(), now_monotonic_ns=1020)
    assert j.stats().duplicate_decisions == 1
    assert j.stats().emitted == 1


def test_capacity_and_expiry_retire_incomplete_pairs():
    j = joiner(max_pending_epochs=1, max_pending_age_ns=100)
    j.add_observation(identity(i_tow=100), observation(), now_monotonic_ns=1000)
    j.add_observation(identity(i_tow=101), observation(), now_monotonic_ns=1010)
    assert j.stats().capacity_evictions == 1
    j.expire_stale(1200)
    assert j.stats().expired_observations >= 1
    assert j.stats().pending_observations == 0


def test_monotonic_watermark_cannot_move_backwards():
    j = joiner()
    j.add_observation(identity(), observation(), now_monotonic_ns=1000)
    with pytest.raises(EpochPairingError, match="watermark"):
        j.add_decision(identity(i_tow=101), decision(), now_monotonic_ns=999)


def test_identity_rejects_noncanonical_source_clock():
    with pytest.raises(EpochPairingError, match="source_clock_id"):
        GnssObservationIdentity(
            process_session_id="join-test",
            connection_generation=2,
            gps_week=2450,
            i_tow_ms=100,
            source_timestamp_ns=1,
            source_clock_id="not-canonical",
            receive_monotonic_ns=1,
        )
