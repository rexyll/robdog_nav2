# libusb transfer lifecycle：fake-transport 状态机与验收合同

- 状态：独立生命周期状态机与合成竞态测试已实现；尚未适配固定版 `ublox_dgnss`，不代表真实 libusb/ROS 2 callback drain 已修复。
- 固定上游审计版本：`aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e`
- 项目分支：`nav` only。

## 1. 上游静态审计确认

固定版本 `usb.cpp` 中：

1. IN/OUT callback 的 `LIBUSB_TRANSFER_NO_DEVICE` 分支提前 return，跳过 `user_data` 持有的 heap-allocated `shared_ptr<transfer_t>` 删除和 completed 标记。
2. `transfer_t::completed` 是普通 bool；callback 写入与 transfer queue 清理/计数读取不共享同步协议。
3. `cleanup_transfer_queue()`、`cleanup_all_transfers()`、`queued_transfer_in_num()` 在 mutex 外先读取 deque size。
4. `cleanup_all_transfers()` 持队列 mutex 调用 `libusb_cancel_transfer()`，随后立即 clear queue。
5. `shutdown()` 进入 `close_devh()` 后未展示 cancel 后持续处理 libusb events 直到所有 completion callback 排空的 barrier。
6. `submit_transfer()` 先调用 `libusb_submit_transfer()`，成功后才把 transfer 放入 bookkeeping queue；若 completion callback 可在登记前执行，则存在登记顺序竞态，需要以 fake transport 和真实事件线程验证。
7. ROS node 通过 wall timer 调用 `handle_usb_events()`；shutdown callback 也可调用 `usbc_->shutdown()`。需确保事件处理和 shutdown 不会并发操作同一个 libusb context/handle。

以上是固定源码的静态证据；尚未声称已通过 TSAN 或设备断连实验复现。

## 2. 状态机

```text
Allocated -> Submitted -> CallbackDrained -> Released
                   |
                   +-> CancelRequested -> CallbackDrained -> Released
```

约束：

- 只有成功提交后才能进入 Submitted。
- cancel 只请求取消，不意味着 callback 已完成，也不授权释放 transfer。
- 正常完成、取消、超时、错误、NO_DEVICE 都必须走同一 completion cleanup 路径。
- callback 必须恰好一次把 Submitted/CancelRequested 转为 CallbackDrained。
- 只有 CallbackDrained（或从未提交的 Allocated）可释放。
- shutdown 先停止新提交，再请求取消；必须继续 pump/drain event handling，直到没有 Submitted/CancelRequested 状态，随后才允许释放 transfer/user_data、关闭 handle/context。
- generation 切换必须在所有旧 generation transfer 已 callback-drained 且 released 后进行。
- 迟到旧 generation callback 即使被判 stale，也必须完成其 transfer cleanup；不能因 stale 而跳过释放。

## 3. 当前独立实现

`navigation/ubx_rtcm_stream/include/robodog_protocol/transfer_lifecycle.hpp`

这是 transport-neutral lifecycle tracker，用于将 ownership/state/barrier 合同从 libusb 中隔离出来。它不调用 libusb，不拥有真实 transfer，不会自动 pump event loop，也不替代 transfer RAII wrapper。

测试 `tests/test_transfer_lifecycle.cpp` 覆盖：

- 禁止 callback 前释放；
- cancel 请求幂等性；
- shutdown 禁止新提交并返回需取消的 transfer；
- callback drain barrier；
- callback 后 release 与 generation rollover；
- stale generation callback 的清理路径；
- duplicate callback 拒绝；
- cancel 与 callback completion 的 200 轮并发竞态。

## 4. 适配固定版驱动前的必需工作

1. 引入 `IUsbTransferTransport` seam，封装 alloc/free/submit/cancel/handle-events/close。
2. 将 `user_data` 裸 `new shared_ptr` 改为单一 owner 的 RAII context；callback 无论 status 均进入统一 finalizer。
3. transfer 在 submit 前登记为 pending；submit 失败必须回滚登记并释放资源。
4. 不在 queue mutex 持有期间调用 cancel、外部 callback、ROS publish 或 libusb event pump。
5. 使用 event-loop owner thread 串行化 libusb context 操作；shutdown 通过 stop-submit/cancel/drain/join/release/close 顺序完成。
6. fake transport 支持可控 callback 顺序、同步/异步 completion、NO_DEVICE、CANCELLED、submit failure、cancel failure 和迟到 callback。
7. 后续验证 ASan/UBSan、TSan、重复 shutdown、disconnect/reconnect、callback 与 shutdown 竞态，以及 ROS 2 Jazzy `component_container_mt` 集成。

**验收边界：当前仅状态机与合成测试实现；没有声称真实 libusb transfer 生命周期、上游驱动或设备验收通过。**
