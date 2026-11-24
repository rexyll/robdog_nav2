"""Pure helpers for deciding whether a ROS 2 topic stream is fresh."""

from enum import Enum
from typing import Optional


class StreamState(str, Enum):
    NOT_RECEIVED = "not_received"
    FRESH = "fresh"
    STALE = "stale"


def classify_stream(
    last_rx_monotonic: Optional[float],
    now_monotonic: float,
    timeout_sec: float,
) -> StreamState:
    """Classify a topic based on local monotonic receive time.

    Monotonic time is used deliberately: ROS time can pause or jump when
    use_sim_time is enabled, while freshness is about elapsed host time.
    """
    if timeout_sec <= 0.0:
        raise ValueError("timeout_sec must be greater than zero")
    if last_rx_monotonic is None:
        return StreamState.NOT_RECEIVED
    age = now_monotonic - last_rx_monotonic
    if age < 0.0:
        return StreamState.NOT_RECEIVED
    return StreamState.FRESH if age <= timeout_sec else StreamState.STALE
