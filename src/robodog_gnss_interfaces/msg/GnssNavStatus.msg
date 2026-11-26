# Decoded UBX-NAV-STATUS fields; raw UBX flag bytes are not exposed by the pinned driver ROSIDL.
uint64 i_tow_ms
uint16 gps_week
string process_session_id
uint64 connection_generation

uint8 gps_fix_type
bool gps_fix_ok
bool diff_soln
bool wkn_set
bool tow_set
bool diff_corr
bool carrier_solution_valid
uint8 carrier_solution
uint8 map_matching_status
uint8 psm_state
uint8 spoof_det_state
uint32 ttff
uint32 msss
