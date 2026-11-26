# Decoded UBX-NAV-COV fields. Covariance terms are the NED upper triangle in m^2.
uint64 i_tow_ms
uint16 gps_week
string process_session_id
uint64 connection_generation

uint8 version
bool pos_cor_valid
bool vel_cor_valid
float64 pos_cov_nn
float64 pos_cov_ne
float64 pos_cov_nd
float64 pos_cov_ee
float64 pos_cov_ed
float64 pos_cov_dd
