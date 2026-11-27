"""Pure decoder for the Linux UAPI struct timex.modes bitmask.

This module decodes the requested mode bits only. It deliberately does not infer
that a clock adjustment was applied, which clock implementation handled it, or
that CLOCK_REALTIME/PHC changed. Those conclusions require target and syscall
result evidence.
"""

from typing import Any


# Values are pinned against Linux v6.12 include/uapi/linux/timex.h.
_MODE_FLAGS = (
    (0x0001, "ADJ_OFFSET", "offset_adjustment_request"),
    (0x0002, "ADJ_FREQUENCY", "frequency_adjustment_request"),
    (0x0004, "ADJ_MAXERROR", "maximum_error_metadata_request"),
    (0x0008, "ADJ_ESTERROR", "estimated_error_metadata_request"),
    (0x0010, "ADJ_STATUS", "clock_status_request"),
    (0x0020, "ADJ_TIMECONST", "pll_time_constant_request"),
    (0x0080, "ADJ_TAI", "tai_offset_request"),
    (0x0100, "ADJ_SETOFFSET", "direct_time_offset_request"),
    (0x4000, "ADJ_TICK", "tick_adjustment_request"),
)

_RESOLUTION_FLAGS = (
    (0x1000, "ADJ_MICRO"),
    (0x2000, "ADJ_NANO"),
)

# These are special multi-bit mode values, not ordinary combinations of flags.
_SPECIAL_MODES = {
    0x8001: (
        "legacy_gradual_offset_request",
        "The legacy adjtime-style request asks for a gradual offset adjustment.",
        False,
    ),
    0xA001: (
        "legacy_remaining_offset_query",
        "The legacy mode requests the remaining gradual offset; it is read-only.",
        True,
    ),
}

_KNOWN_MASK = (
    sum(value for value, _, _ in _MODE_FLAGS)
    | sum(value for value, _ in _RESOLUTION_FLAGS)
)
_UINT32_MAX = 0xFFFFFFFF


def classify_timex_modes(modes: int) -> dict[str, Any]:
    """Decode a uint32 timex.modes value without inferring clock effects.

    Args:
        modes: Raw unsigned 32-bit modes value copied from a valid target ABI.

    Returns:
        A JSON-serializable mapping that preserves the original integer and
        separates operation request bits from resolution modifiers.

    Raises:
        TypeError: if modes is not an integer or is a bool.
        ValueError: if modes is outside the uint32 range.
    """
    if isinstance(modes, bool) or not isinstance(modes, int):
        raise TypeError("modes must be an integer, not bool")
    if modes < 0 or modes > _UINT32_MAX:
        raise ValueError("modes must fit an unsigned 32-bit integer")

    base: dict[str, Any] = {
        "schema_version": 1,
        "source_contract": "Linux v6.12 include/uapi/linux/timex.h",
        "modes": modes,
        "modes_hex": f"0x{modes:08x}",
        "mode_kind": "ordinary_bitmask",
        "classification": "unknown",
        "recognized_mode_flags": [],
        "requested_operations": [],
        "resolution_flags": [],
        "resolution_conflict": False,
        "unknown_bits_mask": 0,
        "unknown_bits_hex": "0x00000000",
        "read_only_request": False,
        "effect_not_proven": True,
        "target_specific_dispatch_required": True,
    }

    special = _SPECIAL_MODES.get(modes)
    if special is not None:
        classification, explanation, read_only = special
        base.update({
            "mode_kind": "special_legacy_value",
            "classification": classification,
            "explanation": explanation,
            "read_only_request": read_only,
        })
        if not read_only:
            base["requested_operations"] = ["legacy_gradual_offset_adjustment"]
        return base

    if modes == 0:
        base.update({
            "classification": "read_only_query",
            "read_only_request": True,
            "explanation": "modes=0 requests clock state without an adjustment mode.",
        })
        return base

    recognized_flags = []
    requested_operations = []
    for value, name, operation in _MODE_FLAGS:
        if modes & value:
            recognized_flags.append(name)
            requested_operations.append(operation)

    resolution_flags = [
        name for value, name in _RESOLUTION_FLAGS if modes & value
    ]
    known_mask = _KNOWN_MASK
    unknown_mask = modes & (~known_mask & _UINT32_MAX)
    resolution_conflict = (
        "ADJ_MICRO" in resolution_flags and "ADJ_NANO" in resolution_flags
    )

    if unknown_mask:
        classification = "contains_unclassified_bits"
    elif requested_operations and len(requested_operations) == 1:
        classification = "single_operation_request"
    elif requested_operations:
        classification = "multiple_operation_request"
    elif resolution_flags:
        classification = "resolution_selection_only_request"
    else:
        classification = "no_recognized_operation"

    base.update({
        "classification": classification,
        "recognized_mode_flags": recognized_flags,
        "requested_operations": requested_operations,
        "resolution_flags": resolution_flags,
        "resolution_conflict": resolution_conflict,
        "unknown_bits_mask": unknown_mask,
        "unknown_bits_hex": f"0x{unknown_mask:08x}",
        "read_only_request": False,
        "explanation": (
            "This is a request-level decode only. The effective behavior depends "
            "on the selected clock implementation, kernel version, validation, "
            "permissions, and syscall execution path."
        ),
    })
    return base
