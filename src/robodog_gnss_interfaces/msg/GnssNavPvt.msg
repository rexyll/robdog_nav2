# Decoded UBX-NAV-PVT epoch. Time is keyed by receiver iTOW, never Header.stamp.
uint64 i_tow_ms
uint16 gps_week
string process_session_id
uint64 connection_generation

# Raw NAV-PVT flags byte; gnssFixOK=bit0, carrSoln=bits7:6.
uint8 raw_flags
