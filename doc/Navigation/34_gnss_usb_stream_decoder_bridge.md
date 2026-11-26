# GNSS USB bulk-IN 到 UBX/RTCM3/NMEA 帧队列桥接

日期：2026-11-26  
分支：nav

## 1. 实现范围

新增 GnssUsbStream，把 DeviceRuntime 与既有增量协议解析器连接起来：

1. 从 DeviceSession 动态发现的 bulk IN endpoint 提交多个异步 read transfer。
2. transfer 完成后将实际收到的字节交给 StreamDecoder；允许 UBX、RTCM3、NMEA 帧跨多个 USB completion 分片。
3. 解码出的完整 Frame 进入 BoundedFrameQueue，由上层非阻塞取帧。
4. 成功/timeout completion 在 stream 仍运行时自动重提交下一笔 IN transfer。
5. Cancelled 不重提交；NoDevice、stall、overflow、error 等 fatal status 停止重提交并请求取消其他 pending transfers。
6. generation 贯穿 transfer callback、decoder Frame 和 bounded queue；重启时 generation 增加、清除旧队列帧并 reset parser partial buffer。

## 2. 默认策略与配置

- IN buffer 默认 12,800 bytes，和已审计的固定上游 transfer 参数一致；仍允许配置。
- 默认 4 个 in-flight bulk-IN transfers。
- 默认 transfer timeout 0（libusb async transfer 不设置超时）。
- 默认 NMEA checksum 不强制；可通过配置启用。
- frame queue 默认容量 256，默认 DropOldest；策略显式配置。
- UBX payload ceiling、decoder buffer、NMEA sentence ceiling 都是配置项，不冒充目标 F9P 的最终部署 whitelist。

## 3. 生命周期与并发

- GnssUsbStream::start/stop 由 lifecycle mutex 串行化。
- transfer completion 在唯一 EventLoop owner thread 执行；decoder 访问另有 mutex 保护，frame queue 自带内部锁。
- stop 先关闭 stream 的 resubmit gate，再调用 DeviceRuntime shutdown；Runtime 停止新提交、cancel pending、pump/drain callbacks、join worker，之后关闭 USB session。
- shutdown timeout 时保留 runtime 与 callback context，调用方必须重试 stop，不能销毁 stream。
- device-lost/error 状态只停止重提交并请求取消其他 pending；自动 hotplug/reconnect 尚未实现。

## 4. Fake USB 测试

test_gnss_usb_stream 使用 fake DeviceSession + fake transfer API：
- 验证 start 后创建多个 pending IN transfers；
- 注入一条带有效 UBX Fletcher checksum 的 byte stream；
- 验证 decoder 生成 UBX Frame，generation 正确；
- 验证 completion 后 IN transfer 自动重提交，in-flight 数量恢复；
- 验证 stop 时 pending transfer 被取消、callback drain、device handle/context 最终释放。

Actions run 45753122570（head 12bddcfd96d31673127afd66f66b434e0f7fa393）四项 job 全部成功，覆盖 GNSS stream 测试及 ASan/UBSan、TSan、Clang libFuzzer、生产 libusb binding。随后新增 lifecycle mutex 的最新代码 head 9f3cd1a0d671119800689700992b2fdbbd5ae1b4 对应 Actions run 45753314359 四项 job 全部成功（ASan/UBSan、TSan、Clang libFuzzer、生产 libusb binding）。最终 README 集成提交 head d6be44a868c49efc0ef16160e52a632293a37c27 对应 Actions run 45753542957 也四项全部成功。

## 5. 验收边界与未完成项

本阶段只证明 synthetic USB transport 到协议 Frame queue 的软件路径。它不证明：
- 实际 ZED-F9P VID/PID/serial、active configuration 和 endpoint topology；
- 真实 libusb 异步 transfer、持续吞吐、USB packet loss 或设备拔插；
- NAV-PVT/TIMEUTC 等 message whitelist 与实际 F9P output configuration；
- GNSS ROS 2 publisher、timestamp mapping、diagnostics 或 lifecycle integration；
- hotplug/reconnect、generation rollover 的真实设备行为。

后续要先锁定设备 MON-VER/descriptor/firmware，再将 Frame queue 接入 GNSS ROS 2 component，并按既有 validity/time mapping 审计规则实现消息解码与发布。


## 7. Ordered ingress lifecycle events and callback identity

`GnssUsbStream` now exposes `try_pop_event()` over one bounded event stream shared by decoded frames and lifecycle transitions. Every event carries immutable `process_session_id`, `connection_generation`, a monotonically increasing `ingress_sequence` assigned while holding the queue mutex, and host `steady_clock` receive time in nanoseconds. Event types are `GenerationStarted`, `Frame`, `DeviceLost`, `TransportFault`, and `Stopped`.

- `process_session_id` is required configuration and must be unique for one process lifetime; it is not guessed from iTOW or ROS Header.stamp.
- `GenerationStarted` is enqueued before the first IN read is submitted.
- Frame events are stamped at USB completion callback entry; all frames parsed from one transfer share that callback receive time while retaining their decoder frame sequence.
- `DeviceLost` / `TransportFault` closes the current generation's frame-ingress gate before remaining transfers are cancelled; callbacks arriving afterward cannot enqueue frames.
- `Stopped` is appended only after transfer callbacks have drained and runtime shutdown succeeds.
- New generations advance connection_generation; old-generation callbacks are rejected. `ingress_sequence` does not reset across generations.
- Frame overflow follows the configured DropOldest/RejectNewest policy. Lifecycle control events take priority and may evict the oldest frame; if a full queue contains only control events, it fails closed and increments `control_event_overflow_failures` rather than silently dropping a lifecycle transition.
- Legacy `try_pop_frame()` is retained only for frame-only consumers and consumes/discards control events. Any consumer implementing reset coordination must use `try_pop_event()` and process the event stream in sequence order.

The event queue and GnssUsbStream synthetic tests passed ASan/UBSan, TSan, libFuzzer smoke, and production libusb binding compile in runs `45760726094` and `45760916265` (four jobs each; 10 CTest tests in sanitizer jobs). Warning-cleanup head `4ed27fb20afd35f08d819078606395a7f6b34e03` passed run `45761176296`: all four jobs succeeded; ASan/UBSan and TSan each passed 10/10 CTest tests; Clang libFuzzer smoke and production libusb binding/native bridge smoke succeeded. The previous multi-character character literal warning is absent from the final logs. This proves the queue/API behavior with fake transport only; it does not prove a real ROS executor/DDS callback barrier or physical USB disconnect behavior.


## 8. Stopped terminal-event delivery and retry semantics

The ingress queue now records `stopped_event_recorded` independently from the general `closed` flag. If a bounded queue is full of lifecycle control events, `close()` returns false rather than claiming that `Stopped` was enqueued. The queue remains closed to frame ingress, but a later `close()` call may retry the terminal marker after the consumer drains an event. Repeated close after a recorded `Stopped` is idempotent. `GnssUsbStream::stop()` propagates this result, so a false return can mean USB runtime shutdown completed but the terminal event still needs to be retried; callers should drain the event queue and call `stop()` again. A capacity-1 synthetic test covers first-close failure, queue drain, successful retry, and idempotence. CI for commits `46da69873793fa5e9610d8080eef24f5963273c7`, `f9f1dd624dc86d22ef8ac326eca019dcdb55941d`, and `1a7d2436e427f8c7e48247343e49d501c26ec8a4` is pending; no pass claim is made yet.


## 9. Per-generation terminal-event idempotence

A generation has at most one terminal cause: the first `DeviceLost` or `TransportFault` wins. Repeated callbacks reporting the same terminal cause are idempotent and do not create duplicate control events; a conflicting later cause is rejected. If the first terminal event cannot be enqueued because the bounded queue contains only control events, its type is retained and the same call may be retried after draining; `close()` also attempts to flush that pending terminal event before appending `Stopped`. A new connection generation cannot replace an active generation; it is admitted only after the previous generation has a recorded terminal event or a recorded `Stopped`. Generation transition admission is atomic: if `GenerationStarted` cannot fit, current_generation is not advanced. Synthetic tests cover duplicate NoDevice-like callbacks, conflicting fatal status, terminal-event retry, active-generation replacement rejection, and failed GenerationStarted admission.


## 10. Reject terminal callbacks after a committed stop

Once `Stopped` has been successfully recorded, later `DeviceLost` or `TransportFault` notifications for that generation are rejected. A terminal callback whose earlier event enqueue failed remains retryable because its terminal type is retained; a queue closed by `close()` without a pending terminal cause does not accept a new terminal cause afterward. This prevents late or duplicate callback paths from appending a contradictory lifecycle event after `Stopped`. The synthetic regression test checks both late terminal event types and confirms the event stream remains `GenerationStarted → Stopped` only.
