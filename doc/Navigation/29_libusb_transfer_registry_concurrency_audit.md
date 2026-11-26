# libusb transfer registry 并发审计与修复记录

日期：2026-11-26  
分支：`nav`  
适用范围：`LibusbTransferTransport` 的 transfer registry。

## 1. 发现的问题

`cancel(id)` 在持有 `operation_mutex_` 时直接查找 `pending_`，但没有取得 `registry_mutex_`。与此同时，`pending_count()` 仅取得 `registry_mutex_`，所以两者可以并发访问同一个 `std::map`。operation mutex 不能保护不获取该 mutex 的读者，因此这是 C++ 容器并发访问数据竞争。

## 2. 修复

`cancel(id)` 现在按固定锁顺序取得：

1. `operation_mutex_`：串行化 cancel 与 submit/event-pump/close 的 native libusb 操作；
2. `registry_mutex_`：保护 pending registry 查找和访问。

其他涉及两个锁的路径也必须遵守 operation → registry 顺序。只读 `pending_count()` 可以单独取得 registry mutex，不会形成反向锁序。

## 3. 测试

增加并发测试：一个线程重复读取 `pending_count()`，另一个线程执行 cancel；随后验证 pending 仍保留到 cancellation callback 被 event pump 派发，最终 drain 后 registry 归零并允许 close。

该测试应在 ThreadSanitizer 下执行。测试源代码已加入现有 `test_libusb_transfer_transport` target；CI 结果尚待本提交对应的 Actions run 确认。

## 4. 尚存的 adapter 设计边界

- `operation_mutex_` 只串行化 native 操作，不强制 event pump 固定由某个 OS thread 执行。生产 DeviceSession 仍必须建立唯一 event-loop owner thread。
- `cancel()` 目前在持有 registry mutex 时调用 native `libusb_cancel_transfer`。当前 callback trampoline 不获取 registry mutex，因此不会形成 callback 等锁环；但真实 libusb adapter 应尽量把 native API 调用移出 registry 锁，采用稳定 transfer 引用/operation serialization，进一步降低锁耦合。
- 当前 API 的 `close_device` 关闭调用方传入的 handle；libusb context 生命周期仍由外部管理。
- 当前 transport 不负责 interface claim/release、kernel driver detach/reattach、descriptor discovery、hotplug、IN resubmit 或 ROS 2 executor lifecycle。

## 5. 验收边界

此处修复只处理 pending registry 的并发访问。只有本提交对应的 TSan CI 成功，才可标记该合成并发测试通过；仍不能据此声称真实设备断连、ROS 2 并发退出或完整 DeviceSession 已验证。
