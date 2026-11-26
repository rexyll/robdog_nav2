# Fake USB transport seam：提交、完成、取消与 shutdown 测试合同

- 状态：可控 fake transport 与 session adapter 已实现；仅为合成 transport 验证，不是 libusb adapter。
- 固定上游：`aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e`
- 目标：在不依赖 F9P、USB 硬件、ROS 2 的条件下，验证 transfer 登记先于 submit、统一 completion finalizer、submit rollback、cancel 失败语义和 shutdown drain 顺序。

## 1. 新增接口

`IUsbTransferTransport` 抽象：
- `submit(id, generation, completion)`
- `cancel(id)`
- `poll_events()`
- `close()`

`FakeUsbTransport`：
- callback 只在 `poll_events()` 中派发，且绝不在内部 mutex 持有时调用用户 callback。
- 支持正常完成、Cancelled、NoDevice、Error。
- 支持一次性 submit failure / cancel failure。
- close 在 pending transfer 或 callback event 尚未清空时拒绝。
- 每个 event 在入队时快照 transfer generation，避免用当前连接代次覆盖旧 transfer 的代次。

`TransferSession`：
- 先 lifecycle register + mark submitted，再调用 transport submit，消除 callback 早于 bookkeeping registration 的窗口。
- submit failure 走独立 SubmitFailed terminal state 并 release，不等待不存在的 callback。
- 所有 terminal callback status 共用一个 finalizer；即使 NoDevice / stale generation 也必须 drain 并 release。
- cancel failure 不等于 transfer 已完成；保留 CancelRequested，继续等待自然完成 callback。
- shutdown 先停止新提交并请求取消，再 pump events；只有 callbacks_drained 后才 close transport。

## 2. 状态机增量

增加 `SubmitFailed` terminal state，用于区分“submit 从未成功，因此不会产生 callback”与“callback 已排空”。SubmitFailed 可 release；shutdown drain 谓词不将其视为 in-flight。

## 3. 测试范围

`tests/test_fake_usb_transport.cpp` 覆盖：
- 正常完成；
- cancel 成功及 Cancelled callback；
- NoDevice / Error / Cancelled 共用 cleanup；
- submit failure rollback；
- cancel failure 后等待自然 completion；
- shutdown 自动 cancel、pump callback、确认 drain 后 close。

## 4. 限制与后续

- fake 不模拟 libusb 内部线程、USB endpoint、event handling timeout 或内核驱动行为。
- `TransferSession` 使用 operation mutex 串行化 submit/cancel/poll/shutdown；fake pending 记录使用 event_queued 防止同一 transfer 重复排队 terminal callback；每个事件保留 submit generation 快照。
- 新增 generation 快照、已排队完成事件与 shutdown 交错、100 轮 submit-vs-shutdown 竞态测试；该批测试的 sanitizer CI 结果待确认。
- 首轮 CI run `45744404049` 的 ASan/UBSan 与 TSan build 因 `submission_failed()` 的字面 `\\n` 转义编译错误失败；修复提交 `29a9efe61f14c84debc59eed25a24ee0b9569c5a` 将其改为正常多行定义，并允许 `SubmitFailed` terminal state release。修复后 CI 尚待确认。
- 下一步增加 callback 与 shutdown 的多线程 stress/TSan，再实现真实 libusb adapter。
- 真实 ROS 2 Jazzy component_container_mt、断连重连和物理 F9P 验收均未完成。

**验收边界：通过本组测试仅表示 fake seam 的状态机语义符合合同，不代表固定版上游驱动缺陷已修复。**
