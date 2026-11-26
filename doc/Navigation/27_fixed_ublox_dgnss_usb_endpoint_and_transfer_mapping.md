# 固定版 ublox_dgnss USB bulk endpoint 与 transfer 参数映射

- 审计对象：`aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e`
- 日期：2026-11-26
- 源码：`ublox_dgnss_node/include/ublox_dgnss_node/usb.hpp`、`ublox_dgnss_node/src/usb.cpp`
- 目的：为 `LibusbTransferRequest` 确定真实上游 bulk IN/OUT 参数，避免猜测 endpoint 和 buffer 配置。

## 1. 固定上游参数

| 参数 | 上游值/来源 | 适配结论 |
|---|---|---|
| IN buffer capacity | `IN_BUFFER_SIZE = 64 * 200 = 12800` bytes | IN request buffer 需预分配 12800 bytes |
| async IN timeout | `libusb_fill_bulk_transfer(..., timeout=0)` | 0 表示不设置 transfer timeout；不可擅自替换为 250 ms |
| async OUT timeout | `libusb_fill_bulk_transfer(..., timeout=0)` | 同样为 0 |
| sync timeout member | constructor 设置 `timeout_ms_ = 250` | 这是同步 read/write timeout，不是当前 async bulk transfer timeout |
| OUT flags | `LIBUSB_TRANSFER_SHORT_NOT_OK` | request.transfer_flags 必须保留此 flag |
| IN flags | 未设置额外 flags | request.transfer_flags 为 0 |
| event pump timeout | `timeout_tv_ = {1, 0}` | event wait 约 1 秒；与单个 transfer timeout 分离 |

## 2. Endpoint discovery

上游没有把 F9P endpoint 固定写成常量，而是在打开设备后读取 active configuration descriptor：

- F9P/F9R CDC-ACM 架构要求 2 个 interfaces。
- 遍历 interfaces 并 detach/claim 每个 interface。
- 对 `LIBUSB_CLASS_DATA`，上游使用 `endpoint[0].bEndpointAddress` 作为 OUT，`endpoint[1].bEndpointAddress` 作为 IN。
- 对 vendor-specific interface，遍历 endpoint descriptor，只选择 `LIBUSB_TRANSFER_TYPE_BULK`，再按 `LIBUSB_ENDPOINT_IN` bit 区分 IN/OUT。
- 这些 descriptor 地址本身已包含 endpoint direction bit；传给 `libusb_fill_bulk_transfer` 时不应丢失或错误重建该地址。

当前 `LibusbTransferRequest` 已显式携带 endpoint address 与 direction，并校验 address bit 7 与 direction 一致；同时增加 `transfer_flags`，由 production binding 写入 native `libusb_transfer::flags`。

## 3. 生命周期职责边界

以下不是 bulk transfer adapter 的职责，必须由后续 device/session 层管理：

- `libusb_init` / `libusb_exit` 与 context ownership
- hotplug attach/detach registration/deregistration
- open/close device handle
- active configuration descriptor 查询与释放
- kernel driver active/detach/reattach
- claim/release interface（F9P/F9R 的 interfaces 0 和 1）
- F9P/F9R CDC control line state 设置
- endpoint descriptor 读取与 generation/session 更新
- attached/keep_running gate 和 callback 后重新提交 IN transfer

特别注意：上游 shutdown 会对 interfaces 0、1 执行 release/reattach；当前 `LibusbTransferTransport::close_device` 只调用 `libusb_close`，不能代替上述 interface/session teardown。

## 4. 下一步适配要求

1. 将 endpoint discovery 的输出形成 immutable device-session configuration，再创建 transfer requests。
2. IN request：endpoint=descriptor-derived IN address、direction=In、buffer size=12800、timeout=0、flags=0。
3. OUT request：endpoint=descriptor-derived OUT address、direction=Out、buffer=payload copy、timeout=0、flags 包含 `LIBUSB_TRANSFER_SHORT_NOT_OK`。
4. 在 callback finalization 后根据 session gate 决定是否 resubmit IN；shutdown/detach 时必须禁止重提交。
5. 在 device/session owner 中按 stop resubmit → cancel outstanding → event callback drain → release interfaces/reattach kernel drivers → close handle → join/exit context 的经验证顺序执行；任何接口释放/关闭不得越过 callback drain。

**当前状态：transfer 参数已从固定源码核对并可由 request 表达；真实 endpoint discovery/device session、CDC control、interface lifecycle 与 ROS 2 集成仍未实现。**
