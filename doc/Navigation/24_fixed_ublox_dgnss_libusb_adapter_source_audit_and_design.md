# 固定版 ublox_dgnss 真实 libusb adapter 源码映射与集成设计

- 审计对象：`aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e`
- 审计日期：2026-11-26
- 范围：`ublox_dgnss_node/src/usb.cpp`、`include/ublox_dgnss_node/usb.hpp`、`src/ublox_dgnss_node.cpp`、CMake/package metadata。
- 本文是固定源码审计与适配器设计，不是上游修复或真实设备验收。

## 1. 构建依赖与版本约束

固定版 `ublox_dgnss_node/CMakeLists.txt` 使用：

- `find_package(PkgConfig REQUIRED)`
- `pkg_check_modules(libusb REQUIRED libusb-1.0)`
- `target_link_libraries(... usb-1.0)`

`package.xml` 声明 `libusb-1.0-dev`，但没有版本下限或精确版本；CMake 也没有检查 `libusb_VERSION`。因此当前驱动源码没有 pin libusb 版本。部署镜像的 apt 仓库、包版本、目标平台及编译时头文件版本必须进入后续 SBOM/build evidence，不能从源码推断具体 libusb 版本。

## 2. 真实 transfer 所有权映射

### 2.1 分配与提交

- `transfer_t` 在 `usb.hpp` 中拥有 `libusb_transfer*`，析构函数直接调用 `libusb_free_transfer()`。
- `transfer_t` 同时持有 `shared_ptr<UCharVector>` buffer；IN/OUT buffer 地址传入 `libusb_fill_bulk_transfer()`。
- `make_transfer_in/out()` 将 `new shared_ptr<transfer_t>(transfer)` 放入 `libusb_transfer::user_data`。
- `submit_transfer()` 先调用 `libusb_submit_transfer()`，成功后才把 shared_ptr 加入 `transfer_queue_`。这在“提交后 callback 可能已发生、queue 尚未登记”的窗口内造成状态观测竞态；user_data 暂时持有对象并不能修复 queue 状态登记顺序。
- submit 失败时当前代码抛异常，但 `user_data` 中新分配的 shared_ptr wrapper 没有在该失败路径显式 delete；必须在 adapter 中把 submit failure 作为“从未进入 libusb ownership”的独立终态并回收 callback context。

### 2.2 callback 与 user_data

- IN/OUT callback 将 `transfer->user_data` cast 为 `shared_ptr<transfer_t>*`，读取/使用后 delete wrapper。
- 正常终态路径把 `completed=true` 写入 `transfer_t` 普通 bool；queue 清理端读取该 bool，二者没有共同 mutex 或 atomic，属于静态可确认的数据竞争风险。
- `LIBUSB_TRANSFER_NO_DEVICE` 在 IN 和 OUT callback 中都提前 return，绕过 debug、completed 标记和 user_data wrapper delete；OUT 的 `LIBUSB_TRANSFER_ERROR` 也提前 return。由此可确认存在 callback context 泄漏和 queue entry 不完成风险。
- callback 在调用上层 IN/OUT handler 后才执行 user_data 清理；callback handler 可能读取 transfer buffer，因此 adapter finalizer 不能早于 payload 消费，也不能在 callback 尚未返回时释放 transfer/buffer。
- IN/OUT callback 可能在末尾 resubmit 新 IN transfer；shutdown gate 必须禁止这一隐式重提交。

### 2.3 queue 与 shutdown

- `cleanup_transfer_queue()`、`cleanup_all_transfers()`、`queued_transfer_in_num()` 在获取 mutex 前先读取 `transfer_queue_.size()`，静态存在锁外 deque 并发访问。
- `cleanup_all_transfers()` 在持有 `transfer_queue_mutex_` 时调用 `libusb_cancel_transfer()`，随后立即 `transfer_queue_.clear()`。由于 cancel 是异步请求，clear 会释放 queue 所持 shared_ptr；若 callback 尚未完成，transfer_t 析构可能释放仍由 libusb 使用的 `libusb_transfer` 和 buffer，属于严重的生命周期风险。当前代码没有 callback-drain barrier。
- `close_devh()` 调用 cleanup 后立即 release interface / close handle；`shutdown()` 随后注销 hotplug callback，析构函数调用 `libusb_exit()`。未看到 event pump 持续运行直至所有 transfer callback drain 的步骤。
- 这组路径的风险来自静态源码顺序；尚未通过真实断连或 ASan 复现，不能声称已观测到实际 UAF/deadlock。

## 3. ROS 2 executor 与 libusb event ownership

- `UbloxDGNSSNode` 为 USB event timer 创建 MutuallyExclusive callback group，10 ms timer 调用 `usbc_->handle_usb_events()`，后者执行 `libusb_handle_events_timeout()`。
- 该 callback group 只约束该 timer callback 自身，不会自动把 USB submit、cancel、shutdown、hotplug 或其他 ROS subscription/service callback 全部串行到同一线程。
- USB 初始化由另一个 wall timer 执行；输入订阅/服务会触发 USB 写操作；transfer completion callback 在 event-handling timer 的执行上下文中触发并可能 resubmit。
- 因此当前结构不是“所有 libusb API 仅由一个线程调用”的模型。真实 adapter 必须明确区分 event handling owner 与 submit/cancel command serialization，并用锁/命令队列/线程约束建立无死锁的调用合同；不能仅因 event timer callback group 是 MutuallyExclusive 就宣称整个 libusb 调用链线程安全。
- shutdown callback 和 node destructor 直接调用 `usbc_->shutdown()`，与 executor timer/callback 的并发退出顺序需要单独处理；仅靠 `keep_running_` 普通 bool 不构成线程同步或 callback drain。

## 4. Adapter 设计决定

### 4.1 保留的边界

保留现有 transport-neutral `IUsbTransferTransport` / `TransferSession` 作为行为合同，但真实 adapter 不直接复用 fake transport 的简化同步假设。真实 adapter 新增独立 `LibusbApi` function table（可注入）和 `LibusbTransferTransport`，把 C API 与生命周期控制分开。

### 4.2 必需的内部对象

每个 transfer 使用稳定的 heap-owned context，至少包含：

- immutable `transfer_id`
- immutable `connection_generation`
- `libusb_transfer*`
- owned byte buffer
- completion callback / owner weak reference
- atomic 或受锁保护的 lifecycle state
- submitted / callback-entered / callback-drained 状态证据

pending registry 必须在 submit 前登记 context。提交失败时仅当 libusb 明确返回同步失败、且 API 合同保证没有 callback 时，才可移除并释放；提交成功后只能由 callback finalizer 释放 transfer context。不得将 `libusb_transfer::user_data` 指向短生命周期栈对象。

### 4.3 线程与锁规则

推荐单一 event-loop owner thread 持续调用 libusb event handling；其他 ROS/executor 线程通过受锁保护的 command ingress 请求 submit/cancel。若后续证据表明目标 libusb API 允许并发 submit/cancel，则仍需规定这些命令与 shutdown gate 的线性化顺序。不得在 registry mutex 内调用 libusb cancel/event handling，也不得持有 callback 需要的锁等待 callback。

callback 只做：校验 context/generation、将传输状态与 payload 结果转交上层、完成统一终态通知。callback 不执行 event pumping、不关闭 handle/context、不在 shutdown gate 关闭后 resubmit。

### 4.4 状态映射

| libusb 事件 | adapter 行为 |
|---|---|
| allocation 成功、尚未 submit | 可由 owner 回收 |
| `libusb_submit_transfer` 同步失败 | `SubmitFailed`，移除 pending 并释放 |
| submit 成功 | 保持 pending ownership，等待 callback |
| `libusb_cancel_transfer` 成功 | 仅标记 cancel requested，继续 event pump |
| cancel 返回 `NOT_FOUND` / `NO_DEVICE` / 其他错误 | 不据此 free；记录错误并继续等待 callback 或经固定版本/平台合同证明的终态 |
| callback status Completed / Cancelled / Error / Stall / Overflow / NoDevice / TimedOut | 全部进入同一个 callback finalizer；状态码只影响上层结果，不改变释放规则 |
| shutdown | stop submit/resubmit → cancel snapshot → event pump → callback drain → join event thread → pending empty assertion → close handle/context |

## 5. 下一阶段实现顺序与验收

1. 定义注入式 `LibusbApi` function table，覆盖 alloc/free、fill/submit/cancel、handle_events、open/close 等 adapter 实际使用的 API。
2. 使用 fake function table 验证同步 submit failure、成功提交、各 callback status、cancel success/failure、callback-before-shutdown、shutdown-during-pending、NO_DEVICE、generation rollover、重复/迟到 callback。
3. 添加 sanitizer 测试；TSan 验证 command ingress、pending registry、shutdown gate；ASan/UBSan 验证 buffer 与 transfer ownership。
4. 只有 adapter 单测稳定后，才在 ROS 2 Jazzy package 中接入，并验证 component_container_mt、executor shutdown 与 node destruction。
5. 最后才进行目标 Linux/libusb 版本 pin、真实 F9P 连续读写、拔插/断连/重连和长时间运行测试。

**当前验收状态：源码审计完成；真实 LibusbTransferTransport 未实现；libusb 版本未 pin；ROS 2 executor integration、真实 USB 与物理 F9P 测试均未完成。**
