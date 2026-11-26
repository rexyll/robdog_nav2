# libusb asynchronous I/O 语义核对与真实 adapter 接入合同

- 固定上游驱动审计对象：`aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e`
- 本文目的：把真实 libusb adapter 的行为约束建立在 libusb 官方异步 I/O API 合同上；不是实现报告。
- 证据来源：
  - https://libusb.sourceforge.io/api-1.0/group__libusb__asyncio.html
  - https://github.com/libusb/libusb/blob/master/libusb/libusb.h

## 1. 官方 API 合同

根据 libusb asynchronous device I/O 文档：

1. transfer 经 `libusb_submit_transfer()` 提交后，completion callback 负责报告成功、timeout、error 或 cancellation 等终态。
2. callback 在 event-handling context 中调用；callback 内不得调用会尝试执行 event handling 的阻塞/重入 API。
3. transfer 只有在 callback 已执行后才可释放；callback 前调用 `libusb_free_transfer()` 属于 undefined behavior。
4. `libusb_cancel_transfer()` 本身是异步、非阻塞操作；函数返回不表示取消已完成。取消真正完成后，libusb 后续调用该 transfer 的 callback，callback 检查 status。
5. 应用必须持续调用 libusb event handling API，让 libusb 处理完成结果并调用 callbacks；libusb 不替应用创建 event-processing thread。

这些要求与项目的 CallbackDrained release gate 一致。

## 2. 对 fake transport 合同的补充

- fake `poll_events()` 是真实 libusb event pump 的替身；真实 adapter 应由单一 event-loop owner thread 驱动 `libusb_handle_events_timeout_completed()` 或等效 event API。
- callback 不负责 event pumping、不调用同步 USB I/O、不关闭 context/handle；只做状态记录、轻量数据转移和 completion notification。
- cancel 返回成功只代表 cancel request accepted；返回 NOT_FOUND / NO_DEVICE 等错误也不能直接作为 transfer 可释放的证据。必须按 libusb API/平台语义确认 callback lifecycle；保守策略是保留 transfer ownership 并继续 event processing，直到 callback/明确的 no-callback terminal path 得到证明。
- callback 收到 `LIBUSB_TRANSFER_NO_DEVICE` 时必须进入统一终态清理；设备断连不能绕过 user_data owner finalizer。
- 若 transfer 需要重新提交，必须明确 callback 中 resubmit 的状态转换与 shutdown gate；shutdown 开始后禁止 resubmit。

## 3. 真实 adapter 推荐结构

```text
ROS node / lifecycle owner
  |
  +-- TransferSession (serialized command interface)
        |
        +-- LibusbTransferTransport
              |
              +-- event-loop owner thread
              |     +-- libusb_handle_events_*(...)
              |     +-- callbacks -> completion finalizer
              |
              +-- pending registry (registered before submit)
              +-- RAII transfer context / user_data owner
```

### Submission

1. Allocate `libusb_transfer` and buffer in RAII context.
2. Construct callback context containing immutable transfer id + connection generation + owner pointer.
3. Register context in pending registry before calling `libusb_submit_transfer()`.
4. If submit fails, remove registry entry and free only because transfer was never submitted and no callback can occur.
5. If submit succeeds, callback becomes the only path that marks completion and releases transfer context.

### Cancellation and shutdown

1. Close submission/resubmission gate.
2. Snapshot in-flight transfers under registry lock; release lock.
3. Call `libusb_cancel_transfer()` without holding registry lock.
4. Continue event processing until every submitted transfer has delivered its callback.
5. Join event-loop owner thread.
6. Verify pending registry empty and all callback contexts released.
7. Only then close device handle and libusb context.

Do not wait for callback while holding a lock that callback needs. Do not call cancel or event-pump functions while holding the pending-registry mutex.

## 4. Explicit unresolved items

- Exact libusb version and platform behavior used by the deployment image are not yet pinned.
- Cancellation return-code handling must be mapped against the pinned libusb version and target OS.
- The upstream driver's current `NO_DEVICE` early-return and queue cleanup defects remain unfixed.
- The actual ROS node timer/event handling ownership must be changed or proven serialized before adapter integration.
- No physical F9P disconnect/reconnect or ROS 2 Jazzy `component_container_mt` integration test has run.

**验收边界：本文为官方 API contract 对照与 adapter design gate；不代表真实 adapter 已实现。**
