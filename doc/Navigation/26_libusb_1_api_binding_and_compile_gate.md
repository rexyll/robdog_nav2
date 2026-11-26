# libusb-1.0 生产 API function-table binding 与编译门禁

- 日期：2026-11-26
- 代码：`navigation/ubx_rtcm_stream/include/robodog_protocol/libusb_api_binding.hpp`
- 实现：`navigation/ubx_rtcm_stream/src/libusb_api_binding.cpp`
- 构建选项：`ROBODOG_BUILD_LIBUSB_BINDING=ON`
- 本文记录生产 API binding 的编译切片，不代表已接入 GNSS ROS 节点或完成设备通信。

## 1. Binding 映射

`make_libusb_api(libusb_context*)` 将注入式 adapter function table 绑定到 libusb-1.0：

| Adapter API | libusb-1.0 |
|---|---|
| alloc_transfer | `libusb_alloc_transfer` |
| free_transfer | `libusb_free_transfer` |
| fill_bulk_transfer | `libusb_fill_bulk_transfer` |
| submit_transfer | `libusb_submit_transfer` |
| cancel_transfer | `libusb_cancel_transfer` |
| handle_events | `libusb_handle_events_timeout_completed` |
| close_device | `libusb_close` |

status mapping 直接使用 libusb transfer status enum；同步 API success 使用 `LIBUSB_SUCCESS`。

## 2. Native callback bridge

libusb callback 签名是 `void(libusb_transfer*)`，而 transport-neutral adapter callback 使用 opaque transfer/status/actual_length/user_data 参数。binding 使用 heap-owned `CallbackBridge` 将二者连接：

1. fill 阶段分配 Bridge，将 adapter callback 和稳定 Context 指针存入其中，并把 Bridge 放进 native transfer 的 user_data。
2. native callback 读取 transfer status/actual_length，清空 native user_data，删除 Bridge，再调用 adapter trampoline。
3. adapter trampoline 只标记 Context callback 状态；transfer 和 Context 仍由 pending registry 持有。
4. `handle_events` 返回后 adapter 才 free native transfer 和通知上层 completion。
5. `libusb_submit_transfer` 同步失败时，binding 删除尚未被 callback 消费的 Bridge；adapter 随后释放 native transfer 和 Context。

此设计避免把 adapter Context 直接作为 libusb user_data 并依赖不兼容的 C callback 函数指针类型，也避免 callback 前释放 transfer。

## 3. Context ownership

`make_libusb_api` 捕获调用方提供的 `libusb_context*` 用于 event handling，但不拥有该 context。调用方必须保证：

- context 在所有 transfer callback drain 前有效；
- transport close 完成后停止并 join event-pump thread；
- 之后才调用 `libusb_exit(context)`。

`close_device` 仅负责 `libusb_close(device_handle)`，不负责 interface release、kernel driver detach/reattach、hotplug registration 或 context teardown。这些职责要在下一阶段与固定上游的设备打开/关闭流程对齐。

## 4. 构建与 CI

默认 standalone parser/fake tests 不要求安装 libusb。启用 `ROBODOG_BUILD_LIBUSB_BINDING=ON` 后，CMake 使用 pkg-config 查找 `libusb-1.0` 并构建 `robodog_libusb_binding`。

新增 GitHub Actions job 安装 Ubuntu runner 的 `libusb-1.0-0-dev`，只编译生产 binding target。该 job 的作用是证明实际 libusb headers 和链接依赖可用，并不运行 USB transfer，也不代表目标部署设备的 libusb 版本已锁定。

## 5. 未完成项

- 未创建 libusb_context/device handle；未执行真实 submit/cancel/event callback。
- 未集成固定版 `ublox_dgnss` 的实际 endpoint、buffer 长度、timeout、重提交和 hotplug 状态机。
- 未实现 event-loop owner thread 和 ROS 2 executor shutdown barrier。
- 未 pin 目标系统 libusb runtime/development package 版本。
- 未完成 F9P 设备连续读写、拔插、断连/重连和长时间压力测试。

**验收状态：生产 libusb API function-table binding 已实现；Actions run `45747349543`（head `e2d8810473cd040c70e605a373416249fbfb1665`）四个 job 全部成功，包括 ASan/UBSan、TSan、Clang libFuzzer 和生产 binding compile。binding compile job 使用 Ubuntu 24.04 runner 的 `libusb-1.0-0-dev 2:1.0.27-1`（pkg-config 报告 libusb 1.0.27），仅作为该 runner 的构建证据，不构成目标部署版本 pin。真实 ROS 节点和硬件集成仍未开始。**
