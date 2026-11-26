# One exact-iTOW GNSS observation after receiver-time + four-stream join.
uint64 i_tow_ms
uint16 gps_week
string process_session_id
uint64 connection_generation

# GPST receiver measurement time, not ROS or host wall time.
uint64 source_timestamp_ns
string source_clock_id
uint64 receive_monotonic_ns

float64 latitude_deg
float64 longitude_deg
float64 altitude_m
int8 navsat_status
bool gnss_fix_ok
uint8 carrier_solution
float64[9] position_covariance
uint8 covariance_type

# UTC is optional metadata; GPST observation remains valid if UTC is untrusted.
bool utc_stamp_valid
int64 utc_seconds
uint32 utc_nanoseconds
bool source_time_accuracy_valid
uint32 source_time_accuracy_ns
bool leap_seconds_valid
int16 leap_seconds
bool leap_source_valid
int16 leap_source
