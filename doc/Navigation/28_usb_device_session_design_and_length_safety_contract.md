# USB DeviceSession 状态机与 libusb transfer 长度边界

日期：2026-11-26  
适用分支：`nav`  
范围：独立导航协议库中的 libusb adapter 与后续设备会话层。

## 1. 当前实现边界

当前 `LibusbTransferTransport` 是 transfer 生命周期适配器，不是完整 USB device session。它接收调用方已经打开的 device handle，并负责单个 bulk transfer 的分配、登记、提交、取消请求、event handling 后的 callback drain 和 transfer 释放。

当前未实现且不得标记为完成的能力：

- libusb context 的创建、唯一所有者和退出；
- VID/PID/serial 枚举与设备身份策略；
- configuration、interface、alternate setting、endpoint descriptor 读取；
- kernel driver detach/reattach 与 interface claim/release；
- CDC-ACM control line state / line coding；
- hotplug、断连后的 generation rollover 和重连；
- IN transfer 自动重提交与停止重提交 gate；
- ROS 2 component/executor 的生命周期集成。

## 2. Transfer length contract

`actual_length` 是 native callback 提供的外部元数据，不可直接信任。对于提交 buffer 容量为 (C) 的 transfer，只有满足：

[
0 \le actual\_length \le C
]

才允许将该长度传给上层。短读允许，数据 vector 必须缩短到实际长度。负值或大于容量的长度均视为 malformed transport completion：统一映射为 `Overflow`，不向上层暴露任何数据，回调长度置零。原始异常值应由未来 telemetry 层另行记录；不得通过截断后仍保留原始长度的方式制造不一致合同。

## 3. DeviceSession 状态机合同

建议状态：

`Created → ContextReady → DeviceOpened → DescriptorsValidated → InterfacesClaimed → Running → StopSubmitting → Cancelling → DrainingCallbacks → InterfacesReleased → DeviceClosed → ContextExited`

错误/断连路径必须进入显式 `Faulted` 或 `DeviceLost`，不得跳过 callback drain 直接 free transfer。

关键不变量：

1. 每个成功 submit 的 transfer 恰好拥有一个稳定 heap context。
2. pending registry 在 submit 前登记；同步 submit failure 才允许无 callback 回滚。
3. cancel 成功只表示请求已接受，不表示 transfer 已终止。
4. 每个成功 submit 最终必须有一个 terminal callback；只有 event handling 返回并完成 drain 后才能 free native transfer 和 buffer。
5. callback 不得执行 event pump、同步 USB I/O、close handle/context 或等待自身 event thread。
6. event pump 由一个 owner thread 驱动；其退出必须晚于所有 pending transfer callback drain。
7. `NO_DEVICE` 是 device-session 终止事件：关闭新提交与 IN resubmit gate，保留旧 generation 的 cleanup，待旧代 transfer drain 后才释放接口和句柄。
8. context 生命周期长于所有 transfer 与 event-pump 调用；device handle 生命周期长于所有引用它的 transfer。
9. Interface claim/release、kernel driver 操作只在 session owner 的串行控制路径执行，不能从 native callback 直接执行。

## 4. 下一步实现顺序

1. 为 libusb context、device handle、claimed interface 建立 RAII owner，但暂不自动探测设备。
2. 引入 `LibusbDeviceApi` function table，覆盖 init/exit、open/close、get_config_descriptor/free_config_descriptor、kernel_driver_active/detach/attach、claim/release interface。
3. 从 descriptor 动态发现 bulk IN/OUT endpoint；CDC-ACM 两接口布局仅作为固定上游的候选路径，不能硬编码为所有设备通用规则。
4. 引入单一 event-loop owner thread；submit/cancel/control operation 由 session command queue 串行化，event thread 只负责 pump 与 native callback dispatch。
5. 增加 IN resubmit policy 与 generation fence；`NO_DEVICE` 时禁止旧代重提交。
6. shutdown 固定为 stop submissions/resubmit → cancel all → pump until callback drain → stop/join event thread → release interfaces/reattach kernel driver → close device → exit context。
7. 以 fake API 注入测试枚举失败、descriptor 不匹配、claim 失败、submit failure、cancel NOT_FOUND、NO_DEVICE、重复断连、重连 generation、shutdown 与 callback 竞态。
8. 通过 sanitizer CI 后，再在 ROS 2 Jazzy component_container_mt 做集成；最终以 ZED-F9P 实物拔插、持续接收、长时间运行和断连恢复证据验收。

## 5. 验收边界

- adapter fake API tests：只证明状态机与内存所有权合同。
- libusb binding compile/smoke：只证明指定 libusb headers/link 与 callback bridge 编译/调用。
- real USB integration：需要真实 USB 总线 I/O 和设备日志。
- ROS 2 integration：需要实际 executor、component unload/shutdown 压测。
- F9P acceptance：需要 MON-VER、固件、协议配置、真实 UBX/RTCM/NMEA 流和拔插恢复日志。

以上层级不可互相替代。
