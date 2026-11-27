import pytest

from robodog_gnss_time.linux_timex_modes import classify_timex_modes


def test_zero_modes_is_a_read_only_query():
    result = classify_timex_modes(0)

    assert result["classification"] == "read_only_query"
    assert result["read_only_request"] is True
    assert result["requested_operations"] == []
    assert result["effect_not_proven"] is True


def test_clockadj_step_modes_are_decoded_as_direct_offset_request():
    result = classify_timex_modes(0x0100 | 0x2000)

    assert result["classification"] == "single_operation_request"
    assert result["recognized_mode_flags"] == ["ADJ_SETOFFSET"]
    assert result["requested_operations"] == ["direct_time_offset_request"]
    assert result["resolution_flags"] == ["ADJ_NANO"]
    assert result["unknown_bits_mask"] == 0


def test_offset_request_is_not_mislabeled_as_a_direct_step():
    result = classify_timex_modes(0x0001 | 0x2000)

    assert result["requested_operations"] == ["offset_adjustment_request"]
    assert result["classification"] == "single_operation_request"


def test_frequency_and_tick_modes_remain_separate_requests():
    result = classify_timex_modes(0x0002 | 0x4000)

    assert result["recognized_mode_flags"] == ["ADJ_FREQUENCY", "ADJ_TICK"]
    assert result["requested_operations"] == [
        "frequency_adjustment_request",
        "tick_adjustment_request",
    ]
    assert result["classification"] == "multiple_operation_request"


def test_status_error_metadata_and_tai_modes_are_not_time_steps():
    result = classify_timex_modes(0x0010 | 0x0004 | 0x0080)

    assert result["requested_operations"] == [
        "maximum_error_metadata_request",
        "clock_status_request",
        "tai_offset_request",
    ]
    assert "direct_time_offset_request" not in result["requested_operations"]


def test_legacy_multibit_modes_are_decoded_before_individual_bits():
    gradual = classify_timex_modes(0x8001)
    remaining = classify_timex_modes(0xA001)

    assert gradual["classification"] == "legacy_gradual_offset_request"
    assert gradual["mode_kind"] == "special_legacy_value"
    assert remaining["classification"] == "legacy_remaining_offset_query"
    assert remaining["read_only_request"] is True


def test_unknown_bits_are_preserved_and_not_silently_ignored():
    result = classify_timex_modes(0x0001 | 0x10000)

    assert result["classification"] == "contains_unclassified_bits"
    assert result["requested_operations"] == ["offset_adjustment_request"]
    assert result["unknown_bits_mask"] == 0x10000
    assert result["unknown_bits_hex"] == "0x00010000"


def test_conflicting_resolution_modifiers_are_explicit():
    result = classify_timex_modes(0x0001 | 0x1000 | 0x2000)

    assert result["resolution_flags"] == ["ADJ_MICRO", "ADJ_NANO"]
    assert result["resolution_conflict"] is True


@pytest.mark.parametrize("value", [True, False, 1.0, "1", None])
def test_modes_rejects_non_integer_values(value):
    with pytest.raises(TypeError):
        classify_timex_modes(value)


@pytest.mark.parametrize("value", [-1, 0x1_0000_0000])
def test_modes_rejects_values_outside_uint32(value):
    with pytest.raises(ValueError):
        classify_timex_modes(value)
