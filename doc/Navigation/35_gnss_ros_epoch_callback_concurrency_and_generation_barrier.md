# GNSS ROS 历元回调并发与接收机代际屏障合同

日期：2026-11-26  
分支：nav  
固定驱动：aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e

## 1. 本轮代码与验证

当前纯 Python GNSS 质量包包含：
- ros_message_adapter.py：将固定驱动 UBXNavPVT、UBXNavHPPosLLH、UBXNavCov、UBXNavStatus 字段转换为 immutable epoch records；忽略 ROS Header.stamp，由调用方提供统一 host monotonic receive timestamp。
- ubx_epoch_joiner.py：按精确 iTOW 关联四种 UBX 消息，要求相同 source_clock_id 与 receiver source timestamp，缓存容量/年龄有界，拒绝冲突重复、迟到片段、已完成历元重复发布和 source-time 回退。
- ros_epoch_adapter.py：等待四路 UBX records 与独立验证的 UbxSourceTimeMapping 均到齐后才输出 observation；RLock 串行保护四路回调、mapping、generation reset 与 cache state。

本轮为 ros_epoch_adapter 增加 RLock，避免 component_container_mt / 多线程 executor 下多路 callback 并发修改同一 pending cache 或重复完成同一历元。增加 ThreadPoolExecutor 五路并发输入测试：NAV-PVT、HPPOSLLH、NAV-COV、NAV-STATUS、time mapping 同时到达，必须恰好一个线程取得唯一 observation，其余返回 None，最终 pending cache 归零。

GitHub Actions：
- run 45755029444，head 4fbbf078bb0f83b657c78c0b87365282769d831e，Humble/Jammy 与 Jazzy/Noble 两个 job 全部 success。
- run 45755029355，同一 head，Humble/Jammy 与 Jazzy/Noble 两个 job 全部 success。
- 这是纯 Python adapter + ThreadPoolExecutor 的单元测试证据，不是实际 rclcpp/rclpy component_container_mt executor、DDS QoS、USB detach/reattach 或设备拔插验证。

## 2. 固定驱动 topic 与运行时事实

固定 launch ublox_rover_hpposllh_navsatfix.launch.py 使用 rclcpp_components/component_container_mt，将 UBX driver 与 NavSatFix convenience node 放入两个独立 multi-threaded component containers。驱动发布 ubx_nav_pvt、ubx_nav_hp_pos_llh、ubx_nav_cov、ubx_nav_status、ubx_nav_time_utc 等原始 UBX topics。

项目质量链路必须消费原始 UBX topics，不消费 convenience fix topic 作为 RTK Fixed 权威来源。上游 convenience node 的 NavSatFix position、covariance、status 是跨 topic 保存的最近状态，不是严格同一 iTOW 的原子历元；不得把它当成项目的四消息精确历元 joiner。

## 3. RLock 解决与未解决的问题

RLock 解决：
- pending epoch OrderedDict 与 retired tombstones 的并发访问；
- mapping 与任意顺序消息回调的并发更新；
- 两个 callback 同时满足完成条件时重复输出；
- reset/begin_generation 与正在执行的 adapter 方法的数据结构竞争。

RLock 不解决：
- 已进入 DDS/executor queue、但在 receiver detach/reset 后才执行的旧消息 callback，被错误地当作新 generation 消息；
- 上游消息本身没有携带项目定义的 process_session_id、connection_generation、queue_sequence；
- USB detach event 与已经排队的 UBX message callback 之间缺少有序 barrier；
- 仅靠 iTOW 无法区分 GPS week rollover、receiver reboot 后 iTOW 重置、或旧消息延迟到达。

因此 begin_generation() 是信任边界，不是自动检测器。生产 ROS integration 在调用 reset/begin_generation 前必须完成以下事务：
1. 停止接受旧连接 generation 的新消息；
2. 将 USB detach/device-lost/reset 事件写入与 UBX frame 同一有序 ingress stream；
3. 等待旧 generation 已排队 callback 全部 drain，或通过显式 generation token 使其不可进入新代；
4. 清空 decoder partial bytes、message caches、time-mapping cache 与 joiner state；
5. 仅在重新验证 MON-VER、GPS week/time validity 和 receiver boot/reset evidence 后开始新 generation；
6. 新 generation 的第一批数据仍需等待可信 NAV-TIMELS/current leap-second source gate；USB attach 本身不恢复 GNSS time validity。

不得只调用 begin_generation(new_id) 然后继续消费旧 DDS queue。若无法证明 callback drain 或 generation identity，则应保持 GNSS quality invalid。

## 4. 后续 ROS runtime 实现门槛

下一阶段应新增真正的 ROS 2 adapter/component，并用 Humble/Jazzy 实际 executor 集成测试：
- 原始 UBX topic subscriptions 使用明确的 QoS profile，并记录实际 publisher/subscriber QoS compatibility；
- callback 入口统一调用同一个 steady/monotonic clock；
- 订阅回调只做严格字段 decode + 入队，不执行长时间阻塞；
- 使用单一串行 epoch worker 或同等原子 transaction，禁止绕过 adapter 直接访问内部 joiner；
- connection generation、process session 和 queue sequence 必须从 USB ingress/lifecycle 事件传递，而非从 iTOW 猜测；
- 测试 MultiThreadedExecutor 并发 callback、callback group、DDS queue 中 reset race、detach barrier、duplicate/reordered samples、shutdown drain；
- source-time mapper 在 NAV-TIMEGPS/NAV-TIMELS raw parser 与 validity/source gate 验收前，不得输出可信 GPS absolute time。

## 5. 验收边界

当前通过的是纯算法层并发单元测试。尚未实现/验证真正的 ROS 2 subscription node、DDS QoS、MultiThreadedExecutor race、device lifecycle barrier、ZED-F9P MON-VER/firmware、NAV-TIMEGPS/NAV-TIMELS 可信 source gate、GPS week rollover 与 reboot 实测。


## 已实现的回调上下文校验补充

四路 UBX ROS epoch record（PVT/HPPOSLLH/COV/STATUS）都携带 process_session_id 与 connection_generation；decoder 必须显式接收上下文。UbxRosEpochAdapter.begin_generation 显式绑定 process/session、connection generation 与 GPS week，并在消息及 time mapping 入队前校验 session/generation。UbxFourMessageEpochJoiner 也验证 process/generation 与 source_clock_id 前缀一致。Actions run `45757990351` 在 Humble/Jammy、Jazzy/Noble 均成功，每环境 133 tests、0 failures。当前记录尚未携带独立 gps_week 字段；GPS week rollover 的 stale callback fence 仍需下一阶段补齐。
