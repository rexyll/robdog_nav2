"""Bounded MID-360 point-cloud UDP sequence and frame-counter audit."""
from __future__ import annotations

from collections import Counter


class LivoxUdpSequenceAudit:
    """Observe udp_cnt in capture order without claiming where a gap occurred.

    The 16-bit counter is reset to zero at a point-cloud frame boundary.
    Pending gaps are bounded so corrupt counters cannot cause unbounded memory use.
    """

    def __init__(self, *, max_pending_gap: int = 4096, recent_window: int = 4096):
        if max_pending_gap < 1 or recent_window < 1:
            raise ValueError("sequence audit bounds must be positive")
        self.max_pending_gap = max_pending_gap
        self.recent_window = recent_window
        self.events = Counter()
        self.missing_estimate = 0
        self.recovered_reorder = 0
        self.pending_missing: set[int] = set()
        self.recent_seen: list[int] = []
        self.recent_set: set[int] = set()
        self.high_water: int | None = None
        self.last_frame_cnt: int | None = None
        self.last_data_type: int | None = None
        self.last_timestamp_ns: int | None = None
        self.frame_counter_deltas = Counter()

    def _remember(self, counter: int) -> None:
        if counter in self.recent_set:
            return
        self.recent_seen.append(counter)
        self.recent_set.add(counter)
        if len(self.recent_seen) > self.recent_window:
            expired = self.recent_seen.pop(0)
            self.recent_set.discard(expired)

    def observe(self, *, udp_cnt: int, frame_cnt: int, data_type: int,
                timestamp_ns: int) -> str:
        if not 0 <= udp_cnt <= 0xFFFF:
            raise ValueError("udp_cnt must fit uint16")
        if not 0 <= frame_cnt <= 0xFF:
            raise ValueError("frame_cnt must fit uint8")

        self.events["observed_packets"] += 1
        event = "first_packet"
        if self.high_water is not None:
            frame_delta = (frame_cnt - self.last_frame_cnt) & 0xFF
            if frame_delta:
                self.frame_counter_deltas[str(frame_delta)] += 1

            if udp_cnt == 0 and self.high_water == 0xFFFF and not frame_delta:
                event = "in_order"
                self.events[event] += 1
                self.high_water = 0
            elif udp_cnt == 0 and self.high_water != 0:
                event = "frame_reset_candidate"
                self.events[event] += 1
                if frame_delta:
                    self.events["frame_counter_changed_at_reset"] += 1
                else:
                    self.events["frame_counter_unchanged_at_reset"] += 1
                self.pending_missing.clear()
                self.high_water = 0
                self.recent_seen.clear()
                self.recent_set.clear()
            elif frame_delta and udp_cnt != 0:
                event = "frame_counter_changed_without_udp_reset"
                self.events[event] += 1
            else:
                delta = (udp_cnt - self.high_water) & 0xFFFF
                if delta == 0:
                    event = "duplicate"
                    self.events[event] += 1
                elif delta < 0x8000:
                    if delta == 1:
                        event = "in_order"
                        self.events[event] += 1
                    else:
                        gap = delta - 1
                        event = "forward_gap"
                        self.events[event] += 1
                        self.missing_estimate += gap
                        self.events["missing_counter_values_estimated"] += gap
                        if gap <= self.max_pending_gap:
                            for step in range(1, delta):
                                self.pending_missing.add((self.high_water + step) & 0xFFFF)
                        else:
                            self.events["untracked_large_gap_values"] += gap
                    self.high_water = udp_cnt
                else:
                    if udp_cnt in self.pending_missing:
                        self.pending_missing.remove(udp_cnt)
                        event = "reordered_gap_recovered"
                        self.recovered_reorder += 1
                        self.events[event] += 1
                    elif udp_cnt in self.recent_set:
                        event = "duplicate"
                        self.events[event] += 1
                    else:
                        event = "out_of_order"
                        self.events[event] += 1

        if self.high_water is None:
            self.high_water = udp_cnt
        self._remember(udp_cnt)

        if (self.last_timestamp_ns is not None
                and timestamp_ns < self.last_timestamp_ns):
            self.events["sensor_timestamp_regressions_in_capture_order"] += 1
        if self.last_data_type is not None and data_type != self.last_data_type:
            self.events["data_type_changes"] += 1

        self.last_frame_cnt = frame_cnt
        self.last_data_type = data_type
        self.last_timestamp_ns = timestamp_ns
        return event

    def report(self) -> dict:
        return {
            "events": dict(self.events),
            "missing_counter_values_estimated": self.missing_estimate,
            "reordered_gap_packets_recovered": self.recovered_reorder,
            "frame_reset_interpretation": (
                "udp_cnt==0 after a nonzero counter is a frame-reset candidate; "
                "frame_cnt is corroborative only when valid for the configured scan mode"
            ),
            "unresolved_gap_values_at_end": len(self.pending_missing),
            "frame_counter_delta_histogram": dict(self.frame_counter_deltas),
            "counter_semantics": (
                "16-bit udp_cnt increments per point-cloud packet and resets to zero "
                "at frame start; 8-bit frame_cnt is not valid for non-repetitive scans"
            ),
            "gap_semantics": (
                "observed sequence gaps in capture order; not proof of network loss; "
                "capture drops, filtering, interface loss, and out-of-order delivery "
                "can affect the observation"
            ),
        }
