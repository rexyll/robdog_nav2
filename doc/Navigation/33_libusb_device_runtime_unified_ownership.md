# libusb DeviceRuntime 顶层统一所有权第一切片

日期：2026-11-26  
分支：`nav`

## 1. 为什么增加顶层 Runtime

DeviceSession、TransferTransport 与 EventLoop 若由调用方分别管理，容易发生 native device handle 被重复关闭或在 pending callback 尚未 drain 时提前释放。新增 `LibusbDeviceRuntime`，统一管理三层对象。

## 2. 所有权关系

`LibusbDeviceRuntime` 拥有：
1. `LibusbDeviceSession`：唯一拥有 libusb context、device handle 和 claimed interface。
2. `LibusbTransferTransport`：拥有每个 transfer、buffer、callback context；仅借用 DeviceSession 的 device handle。
3. `LibusbEventLoop`：拥有唯一 event-pump worker thread。

特别修复了一个潜在 double-close 问题：原 transfer adapter 默认在 `close()` 时调用 `api.close_device`，而 DeviceSession 也负责关闭同一个 handle。现在 `Api::close_device_on_close` 默认 true，保持旧的独立 adapter 行为；Runtime 强制设为 false，使 transport 只停止使用借用的 handle，不关闭它。最后由 DeviceSession 恰好关闭一次。

## 3. Runtime 生命周期

`open(config)`：
- 打开 DeviceSession；
- 将 DeviceSession 的 context 传给 transfer API factory；
- 将 DeviceSession 的 device handle 作为 borrowed handle 传给 transfer adapter；
- 验证 transfer API 完整性；
- 创建并启动唯一 event-loop owner thread。

`shutdown(timeout)`：
- 由 EventLoop 停止新提交、取消所有 pending transfer、持续 pump 直到 callback drain 并 join worker；
- 关闭 TransferTransport（不关闭 borrowed device handle）；
- 关闭 DeviceSession，release interface、reattach kernel driver、close handle、exit context。

若 callback drain 超时，Runtime 保留所有 owner 和 worker，返回 false；不能强行释放资源。析构若仍无法安全 shutdown 则 fail-closed，避免 use-after-free。

## 4. Synthetic 测试

`test_libusb_device_runtime` 覆盖：
- DeviceSession、TransferTransport、EventLoop 组合启动；
- pending transfer 在 shutdown 时取消并派发 terminal callback；
- callback drain 完成后才关闭 DeviceSession；
- borrowed-handle 模式下 TransferTransport 不调用 native close；
- context 最终退出一次。

测试不调用真实 libusb 总线，也不证明设备拔出时的 native callback 完成保证。生产 binding + Runtime 组合仍需在真实 F9P 上做持续读写、断连、重连和 shutdown 压测。

## 5. 当前限制

- Runtime 目前是通用 C++ owner，不是 ROS 2 LifecycleNode。
- command/service 调用尚未进入专用 session command queue；adapter 方法通过 operation mutex 串行化。
- hotplug/reconnect、generation rollover、IN 自动重提交与 parser reset 未实现。
- 真实 VID/PID/serial、active configuration 和 endpoint 拓扑仍需目标设备 MON-VER / descriptor 证据确认。
- 目标 libusb runtime 版本、权限规则、udev 规则尚未锁定。

## 6. CI

Actions run `45752651888`（head `173467273c00a304bdbb0d72955455af113d352c`）四项 job 全部成功：ASan/UBSan、TSan、Clang libFuzzer、生产 libusb binding compile。该 run 覆盖 Runtime 集成测试、borrowed-handle 不重复 close、event-loop owner unbind 前禁止 transport close 等最新代码。测试仍是 fake API/synthetic transport，不代表真实 USB 总线或设备拔插已验收。
