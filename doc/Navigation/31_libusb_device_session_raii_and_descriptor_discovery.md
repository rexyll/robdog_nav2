# libusb DeviceSession RAII 与动态 endpoint discovery 第一切片

日期：2026-11-26  
分支：`nav`  
实现范围：libusb context/device/interface 生命周期、设备身份筛选、active configuration bulk endpoint descriptor discovery。

## 1. 本轮实现

新增：
- `navigation/ubx_rtcm_stream/include/robodog_protocol/libusb_device_session.hpp`
- `navigation/ubx_rtcm_stream/src/libusb_device_session.cpp`
- `navigation/ubx_rtcm_stream/tests/test_libusb_device_session.cpp`

生产 binding：
- `make_libusb_device_api()` 在 `libusb_api_binding.hpp/.cpp` 中实现。
- 使用 libusb device list 按 VID/PID 筛选；若配置 serial number，则打开候选设备并读取 ASCII serial descriptor 精确匹配。
- 读取 active configuration descriptor，枚举 interface alternate settings 下的 bulk endpoints。
- 只选择同一 interface、同一 alternate setting 下同时具有 bulk IN 和 bulk OUT 的 endpoint pair；不硬编码 CDC interface number 或 endpoint address。
- 记录 interface number、alternate setting、endpoint address 和 max packet size。
- RAII session 按顺序 init context → open device → inspect descriptors → kernel driver handling → claim interface → set alternate setting。
- close 按 release interface → reattach previously detached kernel driver → close device → exit context 逆序执行。
- 对 `LIBUSB_ERROR_NOT_SUPPORTED` 的 kernel-driver-active 查询按“平台不支持此查询”处理，不把 macOS 等平台直接判为设备打开失败。

## 2. 失败回滚与测试

Fake API 测试覆盖：
- 正常 open/close 和资源释放顺序；
- VID/PID/serial 参数传递；
- 同接口 IN/OUT endpoint pair 选择；
- IN/OUT 分属不同 interface 时拒绝；
- descriptor 无可用 pair 时回滚；
- interface claim 失败时重新 attach kernel driver 并关闭 device/context；
- kernel-driver query 不受支持的平台路径。

生产 binding smoke test 增加 function-table 完整性检查。以上测试不访问真实 USB 设备，不代表设备枚举、权限、真实 descriptor 或总线 I/O 已实测。

## 3. 生命周期约束

`LibusbDeviceSession` 拥有 context 和 device handle。使用 `native_device()` 构造 `LibusbTransferTransport` 时，调用方必须先停止新提交、cancel 所有 pending transfer、完成 callback drain 并关闭 transport，再关闭 DeviceSession。否则 device handle/context 可能早于 transfer 生命周期结束。

本切片尚未把 transport 和 session 组合为单一 owner 对象，也未实现 event-loop owner thread。生产 session 仍须禁止其他线程绕过 owner 调用同一 libusb context 的 event handling。

## 4. 未完成项

- 当前只读 active configuration；尚未实现 configuration selection / set_configuration。
- 当前要求 bulk IN/OUT 位于同一 interface + alternate setting；若目标设备实际拓扑不同，需要通过 MON-VER/descriptor 实测后扩展 policy。
- 尚未实现 CDC-ACM control line state / line coding。
- 尚未实现唯一 event-loop owner thread、command queue、IN 自动重提交、hotplug、generation rollover 和自动重连。
- 尚未接入 ROS 2 Lifecycle / component_container_mt。
- 未完成 ZED-F9P 实物枚举、连续接收、拔插和长时间运行测试。

## 5. CI 记录

首次完整构建发现测试文件遗漏 `LibusbCancelResult` 的 using 声明，导致 ASan/UBSan 与 TSan build 失败；该错误已在后续提交修正。当前修复后代码的 Actions 结果仍需等待并按最新 head 核验，不能把先前失败或旧 head 的结果记作通过。
