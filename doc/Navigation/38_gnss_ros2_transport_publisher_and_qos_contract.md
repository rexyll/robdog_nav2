# GNSS ROS 2 transport publisher 与 DDS 生命周期合同

## 1. 范围

`robodog_gnss_transport` owns the production `GnssUsbStream` instance and
publishes its ordered `GnssIngressEvent` stream as the ROS 2 message
`robodog_gnss_interfaces/msg/GnssIngressEvent`.

This layer is responsible for transport ownership, one-consumer ordering,
message conversion, subscriber-match gating, QoS declaration, and shutdown
ordering. It does not perform UBX epoch pairing or publish NavSatFix.

## 2. Thread ownership

There are two independent execution owners:

1. libusb event-loop thread, owned by `LibusbDeviceRuntime` / `LibusbEventLoop`;
2. one ROS publisher-consumer thread, the only caller of
   `GnssUsbStream::try_pop_event()` and the only publisher for this stream.

The ROS executor thread does not poll libusb and does not consume the ingress
queue. A single publisher-consumer prevents concurrent consumers from
reordering lifecycle and frame events.

## 3. Subscriber match gate

Before any matched ROS subscription exists, the publisher thread does not pop
events. The bounded C++ ingress queue remains authoritative, so a late initial
subscriber can receive the retained `GenerationStarted` event first. Under
queue pressure, only Frame events may be evicted; lifecycle events are never
silently dropped. Each retained event carries a per-event sequence-prefix
`dropped_frame_events_total` value.

When shutdown begins, the publisher thread drains regardless of subscription
count so that teardown cannot deadlock waiting for a subscriber.

QoS uses reliable, volatile, KeepLast. Volatile durability deliberately does
not replay a completed process session to a late-joining subscriber. If an
earlier subscriber disconnected and a new subscriber joins mid-session, its
first event may have a nonzero sequence; the consumer must reject/reset rather
than assume it received the session prefix. The processor's first-event
GenerationStarted/sequence-zero rule is the fail-closed guard.

## 4. Queue loss and DDS loss are distinct

The ingress queue's sequence-prefix watermark accounts only for DropOldest
Frame evictions that occurred before the current event. It cannot explain
samples lost after the event has been handed to the DDS writer. Consumers
must still enforce strict ingress sequence order and reject unexplained gaps.

The publisher's reliable QoS is a transport request, not proof that an
application subscriber processed a sample. This package currently does not
provide subscriber acknowledgements, DDS resource-limit configuration, or an
application-level delivery receipt. These remain validation gates for
downstream safety-critical use.

## 5. Shutdown order

The node destructor performs the following order:

1. stop new USB submissions and resubmission;
2. cancel pending native transfers and pump until all callbacks drain;
3. close the ingress queue and append `Stopped` if queue capacity permits;
4. signal the publisher-consumer to finish;
5. publish all retained events in queue order and join the publisher thread;
6. destroy the stream/runtime, whose ownership contract releases transport,
   interfaces, device handle, and libusb context only after callback drain.

If callback drain or lifecycle queue closure fails, shutdown is reported as
fatal; the implementation must not pretend that `Stopped` was published.

## 6. Configuration and startup

The node requires explicit nonzero `vendor_id` and `product_id`; optional
`serial_number` narrows selection. It does not assume that every ZED-F9P board
uses one fixed PID or serial layout. The actual target receiver descriptors
must be measured before deployment.

The DDS history depth must be at least the configured ingress queue capacity.
The process session ID is generated at process startup. Host monotonic receive
timestamps remain separate from ROS time, UTC, and GPST.

## 7. Validation and remaining gaps

The first publisher slice contains:
- C++ conversion tests for lifecycle sentinels, UBX/RTCM3/NMEA enum mapping,
  checksum metadata, and frame-generation mismatch rejection;
- a ROS publisher/subscriber delivery test that publishes from a worker thread
  and checks strict event sequence order;
- Humble/Jammy and Jazzy/Noble build/test CI.

Dedicated transport CI run `45771062207` passed on Humble/Jammy and Jazzy/Noble.
Each distribution built the node, passed 6 conversion tests and the reliable
ROS publisher/subscriber worker-thread ordering test; CTest 2/2 and colcon
summary 9 tests, 0 errors, 0 failures, 0 skipped. The subscriber-match gate
was also built/tested on both distributions. This remains synthetic ROS
middleware validation, not a physical USB or receiver integration result.

Not yet validated:
- physical ZED-F9P USB enumeration, permissions, endpoint profile, and sustained
  transfer load;
- subscriber disconnect/reconnect and DDS resource exhaustion;
- end-to-end C++ event to Python processor bridge and subscriber executor
  serialization;
- target firmware, NAV-TIMEGPS/TIMELS output profile, and real leap/time
  synchronization;
- root license and third-party redistribution authorization. The package
  remains `UNSPECIFIED` and this is not an authorization grant.
