# Host clock synchronization evidence. sample_monotonic_ns uses CLOCK_MONOTONIC
# on the same host as the GNSS runtime; it is not a ROS Time or wall-clock stamp.
uint8 STATE_UNKNOWN=0
uint8 STATE_SYNCHRONIZED=1
uint8 STATE_UNSYNCHRONIZED=2
uint8 STATE_FAULT=3

uint8 state
# Source identifier must match the immutable deployment allowlist.
string source_id
# Linux boot ID fences monotonic timestamps across host reboots.
string host_boot_id
# Signed offset convention: positive means host SystemTime is ahead of UTC.
int64 utc_offset_ns
uint64 uncertainty_ns
uint64 sample_monotonic_ns
# Strictly increasing per host boot and monitor instance.
uint64 sample_sequence
