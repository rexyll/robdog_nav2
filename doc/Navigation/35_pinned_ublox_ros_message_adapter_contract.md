# 固定版 ublox_ubx_msgs ROS 消息字段适配合同

日期：2026-11-26  
固定上游：aussierobots/ublox_dgnss @ 9d4f5addbc72d092c57dde99b20712258e96e81e

## 1. 实现

在 robodog_gnss_quality 中新增纯 Python 模块 ros_message_adapter.py，不依赖 rclpy 或生成的 ROS Python 包。它将以下 ROS message 对象转换为现有不可变 epoch records：

- UBXNavPVT：itow、gnss_fix_ok、carr_soln.status；
- UBXNavHPPosLLH：itow、lon/lat/height 与 HP 字段、位置有效性 flags；
- UBXNavCov：itow、pos_cor_valid、六个 NED position covariance 元素；
- UBXNavStatus：itow、gps_fix_ok、carr_soln_valid、carr_soln.status。

Carrier solution 数值映射按固定消息定义：0=NO_CARRIER_RANGE_SOLUTION、1=FLOAT_AMBIGUITIES、2=FIXED_AMBIGUITIES；其余保留/未知值映射 UNKNOWN，不能误判为 Fixed。

## 2. 时间域约束

每个转换函数必须由调用方传入同一 host monotonic clock 的 receive_monotonic_ns。适配器完全忽略 ROS Header.stamp，因为固定上游 USB driver 的 header stamp 是主机接收路径时间，不是 receiver measurement time。

该适配器不构造 source_timestamp_ns，也不推断 source_clock_id。它不允许用 iTOW 单独合成全局时间戳。后续 ROS node 必须等待独立、validated receiver-time mapping（GPS week + receiver boot generation + NAV-TIMELS leap-second validity）后才能把四类消息送入 epoch joiner。

固定上游当前未实现 NAV-TIMEGPS/NAV-TIMELS parser、ROS message 与 F9P output configuration；因此当前时间映射 gate 继续 fail-closed。

## 3. 验证

新增 pytest 覆盖：
- carrier solution 0/1/2/保留值映射；
- HPPOSLLH raw precision integer 与 invalid flags 保真；
- NAV-COV finite values 与 validity bit；
- NAV-STATUS carrier validity 与 GNSS fix flag；
- iTOW 范围、monotonic receive timestamp、ROS bool field 类型；
- NaN covariance 和缺失 nested field 拒绝。

这些是 ROS message-shaped SimpleNamespace fixtures，不是 rclpy subscription/executor 集成测试。没有订阅真实 driver topic，也没有 NavSatFix publisher。

## 4. CI 与分支校验

Actions run 45754335767 对应 nav head 518e0efff52d1785052501e620a0a2b0fa42e721，Humble/Jammy 与 Jazzy/Noble 两个 job 全部 success。该 run 包含 package export、四类 ROS message-shaped fixtures、bool covariance fail-closed 测试。测试并非实际 rclpy 节点运行。

本轮创建测试文件时曾遗漏 create_file 的 branch 参数，导致测试文件短暂写入默认分支；随后已将同一测试文件复制到 nav，并通过默认分支删除提交 89823552ea3283af71c1e17953995f003eb17243 清理。已回读 main 与 nav 的完整 tree：main 不再包含该测试路径，nav 保留测试文件。除该文件的短暂创建/删除外，没有改动 main 的其他文件。
