"""Exact identity join between a GNSS observation and its quality decision.

This module is transport-independent. ROS adapters must derive the same
identity from both messages and pass one shared host-monotonic clock value.
"""

from collections import OrderedDict
from dataclasses import dataclass
from typing import Optional

from .quality_gate import GnssObservation, QualityDecision, RejectionReason
from .ubx_nav_pvt import EpochPairingError


@dataclass(frozen=True)
class GnssObservationIdentity:
    process_session_id: str
    connection_generation: int
    gps_week: int
    i_tow_ms: int
    source_timestamp_ns: int
    source_clock_id: str
    receive_monotonic_ns: int

    def __post_init__(self):
        if not isinstance(self.process_session_id, str) or not self.process_session_id.strip():
            raise EpochPairingError("process_session_id must be a non-empty string")
        if not isinstance(self.source_clock_id, str) or not self.source_clock_id.strip():
            raise EpochPairingError("source_clock_id must be a non-empty string")
        for name, value, maximum in (
            ("connection_generation", self.connection_generation, (1 << 64) - 1),
            ("gps_week", self.gps_week, 32767),
            ("i_tow_ms", self.i_tow_ms, 604799999),
            ("source_timestamp_ns", self.source_timestamp_ns, (1 << 64) - 1),
            ("receive_monotonic_ns", self.receive_monotonic_ns, (1 << 64) - 1),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= maximum:
                raise EpochPairingError(f"{name} is outside its supported unsigned range")
        expected = (
            f"{self.process_session_id}:connection:{self.connection_generation}:"
            f"gps-week:{self.gps_week}"
        )
        if self.source_clock_id != expected:
            raise EpochPairingError("source_clock_id does not match session/generation/GPS week")


@dataclass(frozen=True)
class GnssQualifiedObservation:
    identity: GnssObservationIdentity
    observation: GnssObservation
    quality_decision: QualityDecision


@dataclass(frozen=True)
class GnssObservationQualityJoinerStats:
    emitted: int
    rejected_unqualified: int
    duplicate_observations: int
    duplicate_decisions: int
    conflicting_observations: int
    conflicting_decisions: int
    expired_observations: int
    expired_decisions: int
    capacity_evictions: int
    retired_drops: int
    pending_observations: int
    pending_decisions: int


class GnssObservationQualityJoiner:
    """Bounded exact-key join; only a matching qualified decision emits."""

    def __init__(self, *, max_pending_epochs: int, max_pending_age_ns: int):
        for name, value in (
            ("max_pending_epochs", max_pending_epochs),
            ("max_pending_age_ns", max_pending_age_ns),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        self.max_pending_epochs = max_pending_epochs
        self.max_pending_age_ns = max_pending_age_ns
        self._observations = OrderedDict()
        self._decisions = OrderedDict()
        self._retired = OrderedDict()
        self._watermark_ns = None
        self._emitted = 0
        self._rejected_unqualified = 0
        self._duplicate_observations = 0
        self._duplicate_decisions = 0
        self._conflicting_observations = 0
        self._conflicting_decisions = 0
        self._expired_observations = 0
        self._expired_decisions = 0
        self._capacity_evictions = 0
        self._retired_drops = 0

    @staticmethod
    def _key(identity):
        if not isinstance(identity, GnssObservationIdentity):
            raise EpochPairingError("identity must be a GnssObservationIdentity")
        return (
            identity.process_session_id,
            identity.connection_generation,
            identity.gps_week,
            identity.i_tow_ms,
            identity.source_timestamp_ns,
            identity.source_clock_id,
            identity.receive_monotonic_ns,
        )

    def _advance(self, now_monotonic_ns):
        if (
            isinstance(now_monotonic_ns, bool)
            or not isinstance(now_monotonic_ns, int)
            or now_monotonic_ns < 0
        ):
            raise EpochPairingError("now_monotonic_ns must be a non-negative integer")
        if self._watermark_ns is not None and now_monotonic_ns < self._watermark_ns:
            raise EpochPairingError("joiner monotonic watermark must not move backwards")
        self._watermark_ns = now_monotonic_ns
        for cache, counter_name in (
            (self._observations, "_expired_observations"),
            (self._decisions, "_expired_decisions"),
        ):
            for key, (inserted_ns, _) in list(cache.items()):
                if now_monotonic_ns - inserted_ns > self.max_pending_age_ns:
                    cache.pop(key, None)
                    self._retire(key)
                    setattr(self, counter_name, getattr(self, counter_name) + 1)

    def _retire(self, key):
        self._retired[key] = None
        self._retired.move_to_end(key)
        while len(self._retired) > self.max_pending_epochs * 2:
            self._retired.popitem(last=False)

    def _enforce_capacity(self):
        while len(self._observations) + len(self._decisions) > self.max_pending_epochs:
            obs_first = next(iter(self._observations.items()), None)
            dec_first = next(iter(self._decisions.items()), None)
            if obs_first is None:
                cache, key = self._decisions, dec_first[0]
            elif dec_first is None:
                cache, key = self._observations, obs_first[0]
            elif obs_first[1][0] <= dec_first[1][0]:
                cache, key = self._observations, obs_first[0]
            else:
                cache, key = self._decisions, dec_first[0]
            cache.pop(key, None)
            self._retire(key)
            self._capacity_evictions += 1

    def add_observation(
        self, identity: GnssObservationIdentity, observation: GnssObservation,
        *, now_monotonic_ns: int,
    ) -> Optional[GnssQualifiedObservation]:
        if not isinstance(observation, GnssObservation):
            raise EpochPairingError("observation must be a GnssObservation")
        self._advance(now_monotonic_ns)
        key = self._key(identity)
        if now_monotonic_ns < identity.receive_monotonic_ns:
            raise EpochPairingError("join time precedes observation receive time")
        if (
            observation.source_timestamp_ns != identity.source_timestamp_ns
            or observation.source_clock_id != identity.source_clock_id
            or observation.receive_monotonic_ns != identity.receive_monotonic_ns
        ):
            raise EpochPairingError("observation fields do not match exact epoch identity")
        if key in self._retired:
            self._retired_drops += 1
            return None
        existing = self._observations.get(key)
        if existing is not None:
            if existing[1] == observation:
                self._duplicate_observations += 1
                return None
            self._observations.pop(key, None)
            self._decisions.pop(key, None)
            self._retire(key)
            self._conflicting_observations += 1
            return None
        decision_entry = self._decisions.pop(key, None)
        if decision_entry is not None:
            decision = decision_entry[1]
            self._retire(key)
            if decision.qualified:
                self._emitted += 1
                return GnssQualifiedObservation(identity, observation, decision)
            self._rejected_unqualified += 1
            return None
        self._observations[key] = (now_monotonic_ns, observation)
        self._enforce_capacity()
        return None

    def add_decision(
        self, identity: GnssObservationIdentity, decision: QualityDecision,
        *, now_monotonic_ns: int,
    ) -> Optional[GnssQualifiedObservation]:
        if not isinstance(decision, QualityDecision):
            raise EpochPairingError("decision must be a QualityDecision")
        if not isinstance(decision.qualified, bool):
            raise EpochPairingError("decision.qualified must be bool")
        self._advance(now_monotonic_ns)
        key = self._key(identity)
        if now_monotonic_ns < identity.receive_monotonic_ns:
            raise EpochPairingError("join time precedes observation receive time")
        if key in self._retired:
            self._retired_drops += 1
            return None
        existing = self._decisions.get(key)
        if existing is not None:
            if existing[1] == decision:
                self._duplicate_decisions += 1
                return None
            self._decisions.pop(key, None)
            self._observations.pop(key, None)
            self._retire(key)
            self._conflicting_decisions += 1
            return None
        if decision.qualified and decision.reason != RejectionReason.ACCEPTED:
            raise EpochPairingError("qualified decision must have ACCEPTED reason")
        if not decision.qualified:
            self._observations.pop(key, None)
            self._decisions.pop(key, None)
            self._retire(key)
            self._rejected_unqualified += 1
            return None
        observation_entry = self._observations.pop(key, None)
        if observation_entry is not None:
            self._retire(key)
            self._emitted += 1
            return GnssQualifiedObservation(identity, observation_entry[1], decision)
        self._decisions[key] = (now_monotonic_ns, decision)
        self._enforce_capacity()
        return None

    def expire_stale(self, now_monotonic_ns: int):
        self._advance(now_monotonic_ns)

    def stats(self):
        return GnssObservationQualityJoinerStats(
            emitted=self._emitted,
            rejected_unqualified=self._rejected_unqualified,
            duplicate_observations=self._duplicate_observations,
            duplicate_decisions=self._duplicate_decisions,
            conflicting_observations=self._conflicting_observations,
            conflicting_decisions=self._conflicting_decisions,
            expired_observations=self._expired_observations,
            expired_decisions=self._expired_decisions,
            capacity_evictions=self._capacity_evictions,
            retired_drops=self._retired_drops,
            pending_observations=len(self._observations),
            pending_decisions=len(self._decisions),
        )
