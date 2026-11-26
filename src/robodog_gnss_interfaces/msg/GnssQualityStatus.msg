# Decision for one epoch-aligned GNSS observation; not a NavSatFix or motion command.
bool qualified
string reason
uint32 consecutive_fixed_samples

bool horizontal_std_valid
float64 horizontal_std_m
bool vertical_std_valid
float64 vertical_std_m
bool estimated_speed_valid
float64 estimated_speed_mps

uint64 i_tow_ms
uint16 gps_week
string process_session_id
uint64 connection_generation
uint64 source_timestamp_ns
string source_clock_id
uint64 receive_monotonic_ns
