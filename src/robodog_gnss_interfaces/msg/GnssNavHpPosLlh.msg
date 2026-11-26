# Decoded UBX-NAV-HPPOSLLH epoch; coordinates retain native integer units.
uint64 i_tow_ms
uint16 gps_week
string process_session_id
uint64 connection_generation

int32 lon
int32 lat
int32 height_mm
int8 lon_hp
int8 lat_hp
int8 height_hp

# Raw HPPOSLLH flags byte; invalidLlh=bit0 invalidates all LLH components.
uint8 raw_flags
