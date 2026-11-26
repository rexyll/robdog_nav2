# Ordered transport ingress event. No std_msgs/Header is used because
# receive_monotonic_ns is a host steady-clock value, not ROS/system time.
uint8 EVENT_GENERATION_STARTED=0
uint8 EVENT_FRAME=1
uint8 EVENT_DEVICE_LOST=2
uint8 EVENT_TRANSPORT_FAULT=3
uint8 EVENT_STOPPED=4

uint8 PROTOCOL_UBX=0
uint8 PROTOCOL_RTCM3=1
uint8 PROTOCOL_NMEA=2
uint8 PROTOCOL_NONE=255

uint8 CHECKSUM_NOT_APPLICABLE=0
uint8 CHECKSUM_VALID=1
uint8 CHECKSUM_ABSENT=2

uint8 event_type
string process_session_id
uint64 connection_generation
uint64 ingress_sequence
uint64 receive_monotonic_ns

# Frame fields are populated only for EVENT_FRAME. For lifecycle events use
# PROTOCOL_NONE, CHECKSUM_NOT_APPLICABLE, zero frame sequences, and empty bytes.
uint8 protocol
uint8 checksum_status
uint64 frame_sequence
uint64 frame_connection_generation
uint8[] frame_bytes

# Prefix count of DropOldest frame evictions whose dropped frame sequence
# precedes this event's ingress_sequence. Later drops never rewrite earlier events.
uint64 dropped_frame_events_total
