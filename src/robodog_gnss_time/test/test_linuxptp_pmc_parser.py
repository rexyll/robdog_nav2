import pytest

from robodog_gnss_time.linuxptp_pmc_parser import parse_pmc_response


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
    peerMeanPathDelay 0.0
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
