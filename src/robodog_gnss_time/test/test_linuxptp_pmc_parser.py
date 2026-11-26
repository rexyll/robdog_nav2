import pytest

from robodog_gnss_time.linuxptp_pmc_parser import (
    parse_pmc_response,
    parse_pmc_response_envelope,
    parse_pmc_snapshot,
)


def test_parse_time_status_np_keeps_ptp_master_offset_as_separate_clock_domain():
    output = """sending: GET TIME_STATUS_NP
080027.fffe.7f327b-0 seq 0 RESPONSE MANAGEMENT TIME_STATUS_NP
    master_offset              -98434
    ingress_time               1412169090025854874
    cumulativeScaledRateOffset +1.000000000
    scaledLastGmPhaseChange    0
    gmTimeBaseIndicator        0
    lastGmPhaseChange          0x0000'0000000000000000.0000
    gmPresent                  true
    gmIdentity                 080027.fffe.d9e453
"""
    parsed = parse_pmc_response(output, "TIME_STATUS_NP")
    assert parsed["master_offset"] == -98434
    assert parsed["gmPresent"] is True
    assert parsed["gmIdentity"] == "080027.fffe.d9e453"
    assert str(parsed["cumulativeScaledRateOffset"]) == "1.000000000"
    assert parsed["scaledLastGmPhaseChange"] == 0


def test_parse_current_data_set_decimal_values_as_exact_decimal():
    output = """sending: GET CURRENT_DATA_SET
080027.fffe.7f327b-0 seq 0 RESPONSE MANAGEMENT CURRENT_DATA_SET
    stepsRemoved     1
    offsetFromMaster  42787.0
    meanPathDelay    289207.0
"""
    parsed = parse_pmc_response(output, "CURRENT_DATA_SET")
    assert str(parsed["offsetFromMaster"]) == "42787.0"
    assert str(parsed["meanPathDelay"]) == "289207.0"
    assert parsed["stepsRemoved"] == 1


def test_parse_port_data_set_without_guessing_port_state():
    output = """sending: GET PORT_DATA_SET
080027.fffe.7f327b-0 seq 0 RESPONSE MANAGEMENT PORT_DATA_SET
    portIdentity 080027.fffe.7f327b-0
    portState SLAVE
    logMinDelayReqInterval 0
    peerMeanPathDelay 0
    logAnnounceInterval 1
    announceReceiptTimeout 3
    logSyncInterval 0
    delayMechanism 1
    logMinPdelayReqInterval 0
    versionNumber 2
"""
    parsed = parse_pmc_response(output, "PORT_DATA_SET")
    assert parsed["portState"] == "SLAVE"
    assert parsed["announceReceiptTimeout"] == 3
    assert parsed["logMinDelayReqInterval"] == 0
    assert parsed["logAnnounceInterval"] == 1
    assert parsed["logSyncInterval"] == 0
    assert parsed["delayMechanism"] == 1
    assert parsed["peerMeanPathDelay"] == 0
    assert parsed["versionNumber"] == 2


def test_parse_time_properties_preserves_utc_traceability_flags():
    output = """sending: GET TIME_PROPERTIES_DATA_SET
080027.fffe.7f327b-0 seq 0 RESPONSE MANAGEMENT TIME_PROPERTIES_DATA_SET
    currentUtcOffset 37
    leap61 0
    leap59 0
    currentUtcOffsetValid 1
    ptpTimescale 1
    timeTraceable 1
    frequencyTraceable 1
    timeSource 0x20
"""
    parsed = parse_pmc_response(output, "TIME_PROPERTIES_DATA_SET")
    assert parsed["currentUtcOffset"] == 37
    assert parsed["currentUtcOffsetValid"] is True
    assert parsed["ptpTimescale"] is True
    assert parsed["timeTraceable"] is True
    assert parsed["timeSource"] == "0x20"


@pytest.mark.parametrize("output", [
    "",
    "sending: GET TIME_STATUS_NP\n",
    "080027.fffe.7f327b-0 seq 0 RESPONSE MANAGEMENT TIME_STATUS_NP\nmaster_offset 1",
    """080027.fffe.7f327b-0 seq 0 RESPONSE MANAGEMENT TIME_STATUS_NP
master_offset 1
master_offset 2
gmPresent true
gmIdentity 080027.fffe.d9e453
""",
    """080027.fffe.7f327b-0 seq 0 RESPONSE MANAGEMENT TIME_STATUS_NP
master_offset NaN
gmPresent true
gmIdentity 080027.fffe.d9e453
""",
    """080027.fffe.7f327b-0 seq 0 RESPONSE MANAGEMENT TIME_STATUS_NP
master_offset 1
gmPresent yes
gmIdentity 080027.fffe.d9e453
""",
])
def test_rejects_missing_duplicate_or_malformed_fields(output):
    with pytest.raises(ValueError):
        parse_pmc_response(output, "TIME_STATUS_NP")


def test_rejects_unknown_dataset_and_conflicting_leap_flags():
    with pytest.raises(ValueError):
        parse_pmc_response("anything", "NOT_A_DATASET")
    output = """080027.fffe.7f327b-0 seq 0 RESPONSE MANAGEMENT TIME_PROPERTIES_DATA_SET
currentUtcOffset 37
leap61 1
leap59 1
currentUtcOffsetValid 1
ptpTimescale 1
timeTraceable 1
frequencyTraceable 1
timeSource 0x20
"""
    with pytest.raises(ValueError):
        parse_pmc_response(output, "TIME_PROPERTIES_DATA_SET")


def test_rejects_multiple_responses_for_requested_dataset():
    output = """080027.fffe.7f327b-0 seq 0 RESPONSE MANAGEMENT TIME_STATUS_NP
master_offset 1
gmPresent true
gmIdentity 080027.fffe.d9e453
080027.fffe.7f327b-0 seq 1 RESPONSE MANAGEMENT TIME_STATUS_NP
master_offset 2
gmPresent true
gmIdentity 080027.fffe.d9e453
"""
    with pytest.raises(ValueError):
        parse_pmc_response(output, "TIME_STATUS_NP")


def _snapshot_output(clock_identity="080027.fffe.7f327b"):
    return f"""sending: GET TIME_STATUS_NP
{clock_identity}-0 seq 10 RESPONSE MANAGEMENT TIME_STATUS_NP
    master_offset -20
    gmPresent true
    gmIdentity 080027.fffe.d9e453
sending: GET CURRENT_DATA_SET
{clock_identity}-0 seq 11 RESPONSE MANAGEMENT CURRENT_DATA_SET
    stepsRemoved 1
    offsetFromMaster 20.0
    meanPathDelay 300.0
sending: GET PORT_DATA_SET
{clock_identity}-1 seq 12 RESPONSE MANAGEMENT PORT_DATA_SET
    portIdentity {clock_identity}-1
    portState SLAVE
sending: GET TIME_PROPERTIES_DATA_SET
{clock_identity}-0 seq 13 RESPONSE MANAGEMENT TIME_PROPERTIES_DATA_SET
    currentUtcOffset 37
    leap61 0
    leap59 0
    currentUtcOffsetValid 1
    ptpTimescale 1
    timeTraceable 1
    frequencyTraceable 1
    timeSource 0x20
"""


def test_response_envelope_preserves_clock_port_and_sequence_identity():
    envelope = parse_pmc_response_envelope(_snapshot_output(), "PORT_DATA_SET")
    assert envelope.clock_identity == "080027.fffe.7f327b"
    assert envelope.port_number == 1
    assert envelope.sequence_id == 12
    assert envelope.values["portState"] == "SLAVE"
    with pytest.raises(TypeError):
        envelope.values["portState"] = "MASTER"


def test_snapshot_joins_exactly_four_datasets_by_clock_identity():
    snapshot = parse_pmc_snapshot(_snapshot_output())
    assert snapshot.clock_identity == "080027.fffe.7f327b"
    assert set(snapshot.responses) == {
        "TIME_STATUS_NP", "CURRENT_DATA_SET", "PORT_DATA_SET",
        "TIME_PROPERTIES_DATA_SET",
    }
    assert snapshot.responses["TIME_STATUS_NP"].sequence_id == 10
    assert snapshot.responses["CURRENT_DATA_SET"].sequence_id == 11
    assert snapshot.responses["PORT_DATA_SET"].port_number == 1
    assert snapshot.responses["TIME_PROPERTIES_DATA_SET"].values["currentUtcOffset"] == 37


def test_snapshot_rejects_mixed_responder_clock_identities():
    output = _snapshot_output().replace(
        "080027.fffe.7f327b-0 seq 11",
        "080028.fffe.7f327b-0 seq 11",
    )
    with pytest.raises(ValueError, match="different PTP clock identities"):
        parse_pmc_snapshot(output)


@pytest.mark.parametrize("mutation", [
    lambda output: output.replace(
        "080027.fffe.7f327b-0 seq 13 RESPONSE MANAGEMENT TIME_PROPERTIES_DATA_SET",
        "080027.fffe.7f327b-0 seq 13 RESPONSE MANAGEMENT UNKNOWN_DATA_SET",
    ),
    lambda output: output.split("sending: GET TIME_PROPERTIES_DATA_SET", 1)[0],
    lambda output: output.replace(
        "sending: GET CURRENT_DATA_SET\n",
        "sending: GET CURRENT_DATA_SET\n080027.fffe.7f327b-0 seq 14 RESPONSE MANAGEMENT CURRENT_DATA_SET\nstepsRemoved 1\noffsetFromMaster 20.0\nmeanPathDelay 300.0\n",
    ),
])
def test_snapshot_rejects_missing_duplicate_or_unsupported_dataset(mutation):
    with pytest.raises(ValueError):
        parse_pmc_snapshot(mutation(_snapshot_output()))


def test_response_envelope_rejects_uint16_identity_overflow():
    output = _snapshot_output().replace(
        "080027.fffe.7f327b-0 seq 10",
        "080027.fffe.7f327b-65536 seq 10",
    )
    with pytest.raises(ValueError, match="uint16"):
        parse_pmc_response_envelope(output, "TIME_STATUS_NP")


def test_legacy_parser_keeps_dataset_dictionary_api():
    parsed = parse_pmc_response(_snapshot_output(), "TIME_STATUS_NP")
    assert isinstance(parsed, dict)
    assert parsed["master_offset"] == -20


def test_port_dataset_identity_must_match_response_header():
    output = _snapshot_output().replace(
        "portIdentity 080027.fffe.7f327b-1",
        "portIdentity 080027.fffe.7f327b-2",
    )
    with pytest.raises(ValueError, match="portIdentity"):
        parse_pmc_response_envelope(output, "PORT_DATA_SET")


def test_snapshot_rejects_reused_management_sequence_id():
    output = _snapshot_output().replace(
        "080027.fffe.7f327b-0 seq 13 RESPONSE MANAGEMENT TIME_PROPERTIES_DATA_SET",
        "080027.fffe.7f327b-0 seq 12 RESPONSE MANAGEMENT TIME_PROPERTIES_DATA_SET",
    )
    with pytest.raises(ValueError, match="sequence IDs are not unique"):
        parse_pmc_snapshot(output)


def test_snapshot_compares_clock_identity_case_insensitively():
    output = _snapshot_output().replace(
        "080027.fffe.7f327b-0 seq 11",
        "080027.FFFE.7F327B-0 seq 11",
    )
    snapshot = parse_pmc_snapshot(output)
    assert snapshot.clock_identity == "080027.fffe.7f327b"


def test_port_data_set_parses_signed_log_intervals_as_integers():
    output = _snapshot_output().replace(
        "portState SLAVE",
        "portState SLAVE\\n    logMinDelayReqInterval -3\\n"
        "    logAnnounceInterval 2\\n    logSyncInterval -1\\n"
        "    logMinPdelayReqInterval -4\\n    delayMechanism 2\\n"
        "    peerMeanPathDelay -125\\n    announceReceiptTimeout 3\\n"
        "    versionNumber 2",
    )
    parsed = parse_pmc_response(output, "PORT_DATA_SET")
    assert parsed["logMinDelayReqInterval"] == -3
    assert parsed["logAnnounceInterval"] == 2
    assert parsed["logSyncInterval"] == -1
    assert parsed["logMinPdelayReqInterval"] == -4
    assert parsed["delayMechanism"] == 2
    assert parsed["peerMeanPathDelay"] == -125


def test_port_data_set_rejects_non_integer_log_interval():
    output = _snapshot_output().replace(
        "portState SLAVE",
        "portState SLAVE\\n    logSyncInterval unknown",
    )
    with pytest.raises(ValueError, match="logSyncInterval"):
        parse_pmc_response(output, "PORT_DATA_SET")
