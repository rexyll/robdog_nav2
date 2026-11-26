# Decoded UBX-NAV-HPPOSLLH fields. Coordinates retain native UBX integer units.
uint64 i_tow_ms
uint16 gps_week
string process_session_id
uint64 connection_generation

uint8 version
bool invalid_lon
bool invalid_lat
bool invalid_height
bool invalid_hmsl
bool invalid_lon_hp
bool invalid_lat_hp
bool invalid_height_hp
bool invalid_hmsl_hp
int32 lon
int32 lat
int32 height_mm
int32 hmsl_mm
int8 lon_hp
int8 lat_hp
int8 height_hp
int8 hmsl_hp
uint32 h_acc
uint32 v_acc
