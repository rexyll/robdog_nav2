# libusb 单一 event-loop owner 与 shutdown callback drain 第一切片

日期：2026-11-26  
分支：`nav`

## 1. 本轮目标

此前 adapter 允许任意调用线程执行 `poll_events()`。虽然 `operation_mutex_` 串行化 adapter 方法，但不能建立生产架构所需的唯一 event-loop owner，也不能保证用户 completion callback 不在多个 poll 调用线程间并发派发。

本轮新增 `LibusbEventLoop`，由一个专用线程独占调用 adapter 的 `poll_events()`。

## 2. Adapter 新增合同

`LibusbTransferTransport` 新增：

- `stop_submissions()`：永久关闭新 transfer 提交 gate。
- `cancel_all_detailed()`：对当前 pending registry 中的 transfer 逐一发出异步取消请求，保留每个 transfer 的取消结果；不提前释放任何 transfer。
- `drained()`：只有 pending registry 为空且 `callbacks_in_flight_ == 0` 才返回 true。
- `bind_event_thread()` / `unbind_event_thread()`：绑定 event pump owner thread；绑定后，其他线程调用 `poll_events()` 会被拒绝。

锁顺序保持 operation mutex → registry mutex。native cancel 调用不持 registry mutex。

## 3. LibusbEventLoop 行为

`LibusbEventLoop::start()` 创建 worker，并等待 worker 成功绑定 event-thread ownership 后才返回成功。

`shutdown(timeout)` 顺序：

1. 拒绝从 event worker 自身调用 shutdown，避免 self-join。
2. 停止新提交。
3. 对全部 pending transfer 请求 cancel。
4. 继续由唯一 worker pump libusb events，并等待所有 terminal callback 派发、用户 completion 返回、pending registry 清空。
5. 只有 `drained()` 成立后才停止 worker 并 join。

若 timeout 到期，shutdown 返回 false，event thread 保持运行且 submission gate 保持关闭，调用方可诊断后重试。不能在此状态销毁 event loop、transport 或 DeviceSession。event loop 析构时若 worker 仍 joinable 会 fail-closed，避免静默留下访问已析构对象的线程。

用户 completion 在 event worker 上执行；在 shutdown gate 关闭之前允许 callback reentrant submit，gate 关闭后 reentrant submit 会被拒绝。callback exception 不会逃逸线程入口，记录到 `callback_exception_seen()`，event pump 继续运行。

## 4. Fake API 测试

`test_libusb_event_loop` 覆盖：
- worker 启动完成后才对外报告 start 成功；
- 绑定 owner thread 后，其他线程直接 poll 被拒绝；
- shutdown 停止提交并对 pending transfer 发出 cancel；
- event worker 继续 pump 并派发 terminal callback；
- callback 在 shutdown gate 关闭后尝试 reentrant submit 被拒绝；
- callback drain 完成后 worker join、transport close 成功。

该测试只证明 adapter/event-loop synthetic concurrency contract，不证明 libusb 设备断连时每种 native 错误都一定产生 callback，也不证明 ROS 2 executor 或真实 USB 设备行为。

## 5. 未完成项

- DeviceSession、transfer transport 与 event loop 尚未组合为一个不可拆分的顶层 owner；调用方仍必须严格保证销毁顺序。
- 尚未把 ROS 2 command/service 请求路由到 session command queue；目前 submit/cancel 可由外部线程调用并由 adapter operation mutex 串行化。
- 尚未实现设备热插拔、NO_DEVICE 状态恢复、generation rollover、重连策略。
- 尚未实现固定版 ublox_dgnss 的 IN transfer 自动重提交和重连后 parser generation reset。
- 尚未接入 ROS 2 Lifecycle / component_container_mt，未进行 ZED-F9P 真实拔插测试。

## 6. CI 状态

Actions run `45751870160`（head `e1816a6f00cc2a47586485cff723104a4ce580c0`）四项 job 全部成功：ASan/UBSan、TSan、Clang libFuzzer、生产 libusb binding compile。该 run 覆盖 event-loop worker、submission gate、cancel-all、owner-thread poll guard 与 fake shutdown drain 测试。后续 `19f8b44f13d13c19c7a018316bc703b43f12cdfa` 仅更新 README，相关 run `45752089840` 排队中。真实 USB/ROS 2/F9P 验收仍是后续独立阶段。
