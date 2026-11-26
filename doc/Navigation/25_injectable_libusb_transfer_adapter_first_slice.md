# 注入式 libusb transfer adapter 第一实现切片与测试合同

- 日期：2026-11-26
- 适配对象：固定上游 `aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e`
- 代码：`navigation/ubx_rtcm_stream/include/robodog_protocol/libusb_transfer_transport.hpp`
- 测试：`navigation/ubx_rtcm_stream/tests/test_libusb_transfer_transport.cpp`
- 本文记录的是无 libusb 链接依赖的注入式 adapter seam，不是生产 libusb binding 或真实设备验收。

## 1. 为什么不直接复用现有 IUsbTransferTransport

当前 fake interface 的 `submit(id, generation, completion)` 没有 USB endpoint、方向、buffer 和 timeout，无法完整表达真实 GNSS USB bulk IN/OUT 请求。因此本实现没有伪装成现有 fake interface 的直接替代品，而是先建立具有真实 bulk request 参数的独立 adapter API。后续要统一接口时，应以真实 request contract 为基准，再调整 fake transport 和 TransferSession。

## 2. 当前 API

`LibusbTransferRequest` 包含：

- endpoint address（包含 USB endpoint direction bit）
- `UsbTransferDirection::In/Out`，必须与 endpoint address bit 7 一致
- owned byte buffer；IN 请求按目标接收容量预分配，OUT 请求携带发送 payload
- timeout_ms

`LibusbTransferTransport::Api` 是可注入 function table，包含：

- alloc_transfer / free_transfer
- fill_bulk_transfer
- submit_transfer / cancel_transfer
- handle_events
- close_device
- native success/status code mapping

生产侧可用薄 wrapper 把这些函数绑定到 libusb-1.0 API；单元测试无需安装或链接 libusb。

## 3. 所有权和 callback contract

1. 为 transfer 分配稳定 heap-owned Context，拥有 native transfer、buffer、ID、generation 和 completion。
2. 在调用 native submit 前将 Context 放入 pending registry。
3. fill/submit 同步失败时，确认 native API 未接管 transfer 后移除 registry 并释放；不期待 callback。
4. submit 成功后，transfer 与 Context 继续由 pending registry 持有。
5. native callback 只记录 status / actual_length / callback_seen；不释放 transfer、不调用用户 completion、不执行 event pumping。
6. `poll_events()` 在 operation mutex 下执行 handle_events；handle_events 返回后收集已 callback 的 Context，并释放 native transfer。
7. 用户 completion 在 operation mutex 外执行，允许回调中提交后续请求；callbacks_in_flight 计数阻止并发 close 越过用户 callback 通知。
8. cancel 成功只表示取消请求已发出；pending ownership 保留，直到后续 event pump 派发 callback。
9. NO_DEVICE、ERROR、TIMED_OUT、STALL、OVERFLOW 等状态统一经过 callback drain/finalization，不走提前 return。
10. close 在 pending transfer 或用户 completion 尚未退出时返回 false。析构时若仍无法安全 close 则 fail-fast，防止静默销毁 callback context 并形成悬空 user_data。

## 4. 并发边界

- 所有 adapter API 操作（submit/cancel/poll/close）由 operation mutex 串行化。
- libusb callback 预期只由调用 `handle_events` 的 event-pump 线程派发；当前 adapter 不自行创建 event thread。
- callback trampoline 只在 event-pump 调用栈中写入 Context，`poll_events` 在 handle_events 返回后读取。
- 用户 completion 不在 operation mutex 内调用，避免 callback 重入 submit/cancel 时死锁。
- close 检查 pending registry 与 callbacks_in_flight；不能在回调通知尚未退出时关闭 device handle。
- 生产集成仍需提供唯一 event-loop owner、停止重提交 gate、event thread join，以及 libusb context 的最终释放；当前独立 seam 尚未管理 libusb_context。

## 5. 当前自动化测试覆盖

- 成功提交、Completed callback、actual_length 截断接收 buffer。
- 同步 submit failure 的 transfer 释放与 pending registry rollback。
- cancel 失败后 transfer 保持 pending，随后 NO_DEVICE callback 完成统一清理。
- cancel 成功仅排队取消请求；event pump callback drain 之前 close 被拒绝。
- completion callback 中重入 submit，不因 operation mutex 造成死锁。
- completion callback 执行期间 close 被拒绝；回调退出后可以 close。

测试使用 fake function table，不证明真实 libusb callback、内核 USB、ROS 2 executor 或物理 F9P 行为。

## 6. 已知限制与后续门禁

- 尚无生产 libusb-1.0 function table binding，也未引入 libusb headers / link dependency。
- 当前 `handle_events()` 由调用者驱动；生产集成需单独建立 event-loop owner thread，并确保持续处理 callback。
- 当前 request 只表达 bulk transfer；必须从固定上游逐个映射真实 IN/OUT endpoint、buffer size、timeout、hotplug 和 resubmit 规则。
- 还需验证 callback status code mapping 与目标 libusb 版本一致，且记录 libusb runtime/development package 的精确版本。
- 还未在 ROS 2 Jazzy `component_container_mt` 下集成，也未做真实设备拔插/断连/重连、ASan/TSan 设备压力测试。

**本阶段状态：注入式 adapter 第一实现切片已提交，CI 待本轮最终 commit 结果；生产 libusb binding 和真实设备集成未完成。**
