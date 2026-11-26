# UBX / RTCM3 bounded ingress queue 与 generation fence 合同

- 状态：独立 C++17 queue 已实现；本地 ASan/UBSan 与 ThreadSanitizer 测试通过；GitHub Actions TSan 待验证
- 分支：`nav`
- 适用范围：stream decoder 输出与 ROS/USB 驱动业务消费之间的内存队列；不替代 libusb transfer 生命周期管理。

## 1. 设计目标

1. USB/libusb callback 与 ROS timer 不共享无锁 deque 读写。
2. 队列容量必须有限，且 overflow policy 显式配置；不允许无界增长。
3. 每个 frame 携带 connection_generation；reconnect/reset 后旧 generation frame 必须被拒绝。
4. push / try_pop / stats / close / reset_generation 都使用同一把 mutex 保护。
5. queue 不在锁内执行 ROS publish、日志回调、业务解析或外部函数。
6. 生产 callback 使用非阻塞 push；消费端 move-front + pop 后在锁外处理。
7. 一个 decoder/connection 的队列按协议类型拆分，避免 UBX measurement 与 RTCM correction 在一个队列内互相阻塞；各队列容量和策略按数据 freshness、吞吐与容忍丢失分别定标。

## 2. 已实现 API

文件：`navigation/ubx_rtcm_stream/include/robodog_protocol/bounded_frame_queue.hpp`

- `BoundedFrameQueue(capacity, OverflowPolicy, generation)`
- `push(Frame)`：Accepted / AcceptedAfterDroppingOldest / RejectedFull / RejectedStaleGeneration / RejectedClosed
- `try_pop()`：返回拥有独立生命周期的 Frame；无数据时返回空 optional
- `reset_generation(new_generation)`：原子清空旧队列、切换 generation 并重新打开 queue
- `close()`：拒绝新 push，但保留已有帧供消费端 drain
- `clear()`：清空待消费帧
- `stats()`：accepted、popped、drop/reject counters、current_size、high_watermark、closed、generation

Overflow policy:
- DropOldest：保留最新数据，显式统计被淘汰的旧帧。
- RejectNewest：保留已排队帧，拒绝新帧。
- 不预设哪一种适用于全部协议；RTCM correction 与 UBX measurement 的数据陈旧度/连续性语义须在集成层分别评估。

## 3. Generation 与 shutdown 合同

- detach/reconnect 时先停止新 transfer submission。
- 将队列 close，禁止迟到 callback 继续 push；callback 若仍运行，需通过 USB transfer ownership/barrier 确保其生命周期。
- 取消 outstanding libusb transfer，并继续 pump/drain event handling，直到所有 cancel completion callback 完成。
- join USB event thread，确认没有 callback 仍引用 transfer/user_data/device handle。
- 释放 transfer/user_data 与 libusb handle/context。
- 连接重新建立后推进 generation，调用 decoder reset 与 queue reset_generation；旧 generation 数据必须拒绝。
- 以上是目标生命周期顺序；当前 bounded queue 只实现队列侧 close/reset fence，不代表 libusb cancel callback drain 已实现。

## 4. 测试与验证

`tests/test_bounded_frame_queue.cpp` 覆盖：
- RejectNewest FIFO/统计；
- DropOldest FIFO/统计；
- generation reset 清空旧帧并拒绝 stale frame；
- close 后拒绝 push、允许 drain；
- 4 producer + 1 consumer 并发传输 4000 frames；
- 多 producer overflow 下容量上限与 drop accounting。

本地验证（GCC 14.2）：
- ASan/UBSan：CTest 2/2 passed（stream parser + bounded queue）。
- ThreadSanitizer：CTest 2/2 passed，`TSAN_OPTIONS=halt_on_error=1`，无 data-race 报告。

GitHub Actions 的 TSan job 尚待实际运行；本地合成 queue 测试不等同于上游驱动的 TSAN 验收。

## 5. 集成前未决项

- 以实际 ZED-F9P output whitelist、消息频率、最大 payload、允许排队延迟和内存预算计算容量，不在本库默认猜测生产值。
- 定义 UBX 与 RTCM 独立 overflow policy；RTCM frame drop 可能影响 correction continuity，不能只按“最新优先”直接决策。
- parser sequence / generation 必须贯穿 queue、ROS message wrapper、diagnostic counters。
- 原上游 `transfer_queue_` 中 completed 普通 bool、锁外 deque size 读取、NO_DEVICE early return cleanup、cancel callback drain/barrier 仍需单独修复。
- ROS 2 Jazzy MultiThreadedExecutor 下的 component integration 与 USB disconnect/reconnect stress test 尚未实施。

**结论：独立队列原语与并发测试已建立；上游 libusb transfer ownership/lifecycle 与 ROS executor 集成仍未验收。**
