# Receiver-time mapping derived only from a matched NAV-TIMEGPS/NAV-TIMELS pair.
# source_timestamp_ns is receiver GPST in the declared source_clock_id domain.
# receive_monotonic_ns is host steady-clock time and is not ROS time.
uint64 i_tow_ms
uint64 source_timestamp_ns
string source_clock_id
uint64 receive_monotonic_ns
bool valid
bool utc_stamp_valid
bool utc_seconds_valid
int64 utc_seconds
bool utc_nanoseconds_valid
uint32 utc_nanoseconds
string validity_reason
bool gps_week_valid
uint16 gps_week
bool source_time_accuracy_valid
uint32 source_time_accuracy_ns
bool leap_seconds_valid
int16 leap_seconds
bool leap_source_valid
int16 leap_source
bool reported_leap_seconds_valid
int16 reported_leap_seconds
bool leap_event_valid
bool time_to_leap_event_seconds_valid
int32 time_to_leap_event_seconds
string process_session_id
uint64 connection_generation
