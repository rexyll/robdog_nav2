# Decoded UBX-NAV-PVT fields; raw UBX flag bytes are not exposed by the pinned driver ROSIDL.
uint64 i_tow_ms
uint16 gps_week
string process_session_id
uint64 connection_generation

uint8 gps_fix_type
bool gnss_fix_ok
bool diff_soln
uint8 carrier_solution
bool invalid_llh
