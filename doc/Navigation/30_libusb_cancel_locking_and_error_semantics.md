# libusb cancel 锁耦合与错误语义审计

日期：2026-11-26  
分支：`nav`  
适用范围：`LibusbTransferTransport::cancel()` 与 libusb-1.0 binding 的异步取消边界。

## 1. 本轮发现与调整

此前 `cancel(id)` 同时持有 `operation_mutex_` 和 `registry_mutex_` 调用 native `libusb_cancel_transfer()`。虽然当前 callback trampoline 不获取 registry mutex，但这会让只读取 registry 的观测者（例如 `pending_count()`）被 native API 调用阻塞，并把 registry 临界区扩大到外部库调用。

现在改为：

1. 取得 `operation_mutex_`，串行化本 adapter 的 submit、event pump、cancel 与 close。
2. 在 `registry_mutex_` 内查找 transfer 并复制稳定的 native transfer 指针。
3. 释放 `registry_mutex_`，仍持有 `operation_mutex_` 时调用 native cancel。
4. 保持 transfer 在 `pending_` 中，直到 event pump 派发 callback 并完成 drain；cancel 返回值本身不释放 transfer。

该指针稳定性依赖一个明确前提：所有能派发该 context callback 的 event handling 都必须由本 adapter 的 operation serialization 控制。生产 DeviceSession 仍必须把 event pump 固定在唯一 owner thread；禁止绕过 session 从其他线程直接对同一 libusb context 调用 event handling。

## 2. 取消结果分类

新增 `LibusbCancelResult`：

- `Requested`：native cancel 返回成功，只表示取消请求被接受。
- `NotPending`：adapter registry 中没有可取消的 pending transfer。
- `NotFound`：native libusb 返回 `LIBUSB_ERROR_NOT_FOUND`；不能据此释放仍由 adapter 持有的 transfer，继续等待其终态 callback/drain 或由 session 的故障策略处理。
- `DeviceLost`：native libusb 返回 `LIBUSB_ERROR_NO_DEVICE`；向 DeviceSession 提供设备丢失信号，但 pending ownership 仍保留，直到 callback drain。
- `Failed`：其他 native 错误或 adapter 已关闭。

保留 bool `cancel(id)` 兼容接口，仅当结果为 `Requested` 时返回 true。需要区分设备丢失、not-found 与普通失败的上层应调用 `cancel_detailed(id)`。

## 3. 新增测试

- native fake cancel hook 内同步调用 `pending_count()`，验证 native cancel 调用期间不持有 registry mutex；若锁仍被持有，该测试会死锁。
- 保留 20,000 次并发 registry 读取与 cancel 测试。
- 注入 `-5`（libusb `LIBUSB_ERROR_NOT_FOUND`）和 `-4`（libusb `LIBUSB_ERROR_NO_DEVICE`），验证结果分类且 pending ownership 不被提前释放。
- 通过后续 callback/event pump 完成 drain，再允许 close。

## 4. 尚未解决的约束

- `operation_mutex_` 只串行化 adapter 自身的方法，不会阻止外部代码绕过 adapter 调用 `libusb_handle_events*`。唯一 event-loop owner 必须由 DeviceSession 架构强制执行。
- 当前 `poll_events()` 在释放 operation mutex 后调用用户 completion；这支持 callback reentrant submit，但多个调用线程仍可能在不同 poll 调用中并发执行用户 completion。生产 session 应由单一 event owner 执行 poll，并明确 completion 的线程语义。
- `LIBUSB_ERROR_NOT_FOUND` 和 `LIBUSB_ERROR_NO_DEVICE` 的分类不会取代 callback drain；不得在 cancel 返回后直接 free transfer、buffer、context 或 device handle。
- 还未实现 context/device/interface RAII、descriptor endpoint discovery、kernel driver detach/reattach、hotplug、自动重连、ROS 2 lifecycle 集成和 ZED-F9P 设备测试。

## 5. 验收边界

本轮代码和 synthetic fake API 测试已提交。只有最新代码 head 对应的 ASan/UBSan、TSan、libusb binding CI 全部成功，才能将这些测试记为 CI 通过。该测试仍不等于真实 USB 总线取消、设备拔插或 ROS 2 executor 运行态验证。
