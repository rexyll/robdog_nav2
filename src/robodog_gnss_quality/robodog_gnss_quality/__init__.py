"""RTK quality gating primitives for the RobDog Nav2 project."""

from .quality_gate import (
    CarrierSolution,
    CovarianceType,
    GnssObservation,
    GateConfig,
    QualityDecision,
    RejectionReason,
    RtkQualityGate,
)
from .ros_epoch_adapter import (
    UbxRosEpochAdapter,
    UbxRosEpochResult,
    UbxSourceTimeMapping,
)
from .ros_message_adapter import (
    decode_nav_cov_message,
    decode_nav_hp_pos_llh_message,
    decode_nav_pvt_message,
    decode_nav_status_message,
)
from .ubx_nav_pvt import (
    EpochPairingError,
    NavSatEpochRecord,
    UbxNavPvtQuality,
    decode_nav_pvt_flags,
    pair_navsat_fix_with_nav_pvt,
)

__all__ = [
    "CarrierSolution",
    "CovarianceType",
    "GnssObservation",
    "GateConfig",
    "QualityDecision",
    "RejectionReason",
    "RtkQualityGate",
    "EpochPairingError",
    "NavSatEpochRecord",
    "UbxNavPvtQuality",
    "decode_nav_pvt_flags",
    "pair_navsat_fix_with_nav_pvt",
    "decode_nav_pvt_message",
    "decode_nav_hp_pos_llh_message",
    "decode_nav_cov_message",
    "decode_nav_status_message",
    "UbxRosEpochAdapter",
    "UbxRosEpochResult",
    "UbxSourceTimeMapping",
]
