# Decoded UBX-NAV-STATUS epoch. Preserve raw status bytes for audit.
uint64 i_tow_ms
uint16 gps_week
string process_session_id
uint64 connection_generation

# NAV-STATUS flags.gpsFixOK=bit0; fixStat.carrSoln=bits2:1.
uint8 raw_flags
uint8 raw_fix_stat
