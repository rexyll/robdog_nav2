# robodog_gnss_transport

ROS 2 publisher node that owns the `GnssUsbStream` runtime and publishes its
ordered ingress events as `robodog_gnss_interfaces/msg/GnssIngressEvent`.

## Current behavior

- Selects a USB receiver by explicitly configured VID/PID and optional serial.
  No receiver identity is assumed by default.
- Uses the single-owner libusb event-pump thread provided by
  `LibusbDeviceRuntime` / `LibusbEventLoop`.
- Uses a separate, single publisher-consumer thread. It is the only consumer
  of `GnssUsbStream::try_pop_event()` and publishes events in ingress sequence
  order.
- Publishes with reliable, volatile, KeepLast QoS. `dds_history_depth` must be
  at least `frame_queue_capacity`. Before the first matched subscriber, the
  publisher thread leaves events in the bounded ingress queue so the initial
  `GenerationStarted` is retained; DropOldest frame loss is represented by the
  per-event sequence-prefix watermark. During shutdown it drains even without
  a matched subscriber. A subscriber that joins after an earlier subscriber
  disconnected must treat a nonzero initial sequence as a discontinuity and
  reset/fail closed; volatile QoS does not replay a completed process session.
- On shutdown, stops USB submissions and drains native callbacks first,
  closes the ingress queue with `Stopped`, then drains all retained ingress
  events on the publisher thread before joining it.
- Uses a host-generated process session ID and preserves host steady-clock
  receive timestamps. These values are not ROS time, UTC, or GPS time.

## Parameters

| Parameter | Default | Constraint |
|---|---:|---|
| `vendor_id` | required | 1..65535 |
| `product_id` | required | 1..65535 |
| `serial_number` | empty | optional exact serial filter |
| `event_topic` | `/gnss/ingress` | non-empty |
| `frame_queue_capacity` | 256 | 1..65536 |
| `dds_history_depth` | 256 | at least ingress capacity |
| `read_buffer_bytes` | 12800 | 1..16777216 |
| `in_flight_reads` | 4 | 1..64 |
| `event_poll_timeout_ms` | 50 | 1..1000 |
| `require_nmea_checksum` | false | boolean |

Run the node with explicit receiver descriptors, for example:

```bash
ros2 run robodog_gnss_transport gnss_usb_transport_node --ros-args \
  -p vendor_id:=0x1546 -p product_id:=0x01a9
```

The VID/PID in this example is illustrative only; verify the target receiver's
USB descriptors before deployment.

## Validation boundary

Message conversion unit tests and ROS package CI validate mapping and builds.
They do not establish real receiver enumeration, permissions/udev configuration,
physical USB stability, DDS subscriber delivery under overload, or ZED-F9P
firmware compatibility.

The package license is `UNSPECIFIED` because the repository has no root
license declaration. This is a governance blocker, not permission to
redistribute binaries or source under an assumed license.
