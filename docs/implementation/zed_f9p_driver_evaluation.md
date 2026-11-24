# ZED-F9P ROS 2 驱动评估与历元配对决策

- 日期：2026-11-24
- 状态：**首选候选已选，进入跨发行版构建验证；未批准实机接入**
- 项目进度唯一台账：[根目录 TaskPlan.md](../../TaskPlan.md)
- 候选驱动：[aussierobots/ublox_dgnss](https://github.com/aussierobots/ublox_dgnss)
- 固定版本：tag `0.7.6`，commit `9d4f5addbc72d092c57dde99b20712258e96e81e`
- 上游许可证：Apache-2.0（仓库 LICENSE、各 ROS package.xml 均声明 Apache 2.0）

## 1. 决策摘要

项目 ZED-F9P 的 ROS 2 driver 首选候选为 `aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e`。这是**项目自己的适配与测试基线**，不是 u-blox 官方驱动背书，也不是已经完成的硬件验收。

选择理由：

1. 上游 README 明确列出 ZED-F9P，并声明支持 Ubuntu 22.04/24.04、ROS 2 Humble/Jazzy。
2. 原始 UBX ROS 消息保留 GNSS 周内历元 `iTOW`：`UBXNavPVT`、`UBXNavHPPosLLH`、`UBXNavCov`、`UBXNavStatus` 都有自己的 `itow` 字段。
3. `UBXNavPVT` 直接给出 `gnss_fix_ok` 与 `carr_soln.status`，无需从通用 `NavSatStatus` 猜 RTK Fixed/Float。
4. `UBXNavHPPosLLH` 保留高精度经纬度/高度分量；`UBXNavCov` 明确提供同一历元的 NED 位置协方差和 `pos_cor_valid` 标志。
5. Apache-2.0 且源代码固定到 tag 对应 commit，便于跨 Humble/Jazzy 重复构建、SBOM 与后续升级审查。

这只是对当前公开源码/API 的静态审查结果。仓库新增的 `.github/workflows/ublox-dgnss-driver-ci.yml` 用来进一步验证该固定来源能否在项目 Humble/Jazzy 环境编译；CI 成功仍不等价于真实 ZED-F9P USB、固件配置、RTK 改正或 PPS 工作正常。

## 2. 固定来源与关键接口

| 上游组件 | 版本 | 本项目用途 |
|---|---|---|
| `ublox_dgnss_node` | 0.7.6 | 通过 USB 读取 F9P UBX 数据并发布原始 UBX ROS 消息 |
| `ublox_ubx_msgs` | 0.7.6 | 提供含 `iTOW`、PVT 质量标志、HPPOSLLH、NAV-COV、NAV-STATUS 的消息接口 |
| `ublox_nav_sat_fix_hp_node` | 0.7.6 | 上游可选 NavSatFix 转换节点；本项目不把它的输出直接作为质量门控输入 |
| `ntrip_client_node` | 0.7.6 | 可选 RTCM/NTRIP 转发组件；具体服务商和改正链另行确认 |

固定源码链接：
- [tag 0.7.6 / commit 9d4f5addbc72d092c57dde99b20712258e96e81e](https://github.com/aussierobots/ublox_dgnss/tree/9d4f5addbc72d092c57dde99b20712258e96e81e)
- [上游 README](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/README.md)
- [UBXNavPVT.msg](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_ubx_msgs/msg/UBXNavPVT.msg)
- [UBXNavHPPosLLH.msg](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_ubx_msgs/msg/UBXNavHPPosLLH.msg)
- [UBXNavCov.msg](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_ubx_msgs/msg/UBXNavCov.msg)
- [UBXNavStatus.msg](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_ubx_msgs/msg/UBXNavStatus.msg)
- [UBXNavPVT publisher implementation](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_dgnss_node/src/ublox_dgnss_node.cpp)

### 2.1 必须采用的严格同历元订阅策略

项目适配层订阅上游原始消息，而不是依赖两个 topic 的“最近到达”关系：

- `UBXNavPVT`: 质量字段 `gnss_fix_ok`、`carr_soln.status`，以及 `iTOW`。
- `UBXNavHPPosLLH`: 高精度位置分量、无效标志和 `iTOW`。
- `UBXNavCov`: `pos_cor_valid`、六个 NED 上三角协方差元素和 `iTOW`。
- `UBXNavStatus`（建议启用）: `gps_fix_ok`、`carr_soln`、`iTOW`、`msss`。其中 `msss` 是 receiver startup/reset 后的毫秒计数，可辅助识别重启或时间状态复位，但自身也需要处理计数回绕。

一个可进入质量门控的完整位置样本，至少要求 PVT 与 HPPOSLLH 的 `iTOW` 完全相同；如果采用 NAV-COV，则 covariance 也必须在相同 `iTOW` 且 `pos_cor_valid=true`。任何不匹配、缺失、超出缓存窗口或重复冲突样本都应丢弃，不允许 nearest-time 猜配。

`UBXNavCov` 的坐标系是 NED，而项目 `NavSatFix.position_covariance` 契约为 ENU。转换应显式使用轴置换/符号矩阵：

```text
R_NED_to_ENU = [[0, 1,  0],
                [1, 0,  0],
                [0, 0, -1]]
C_ENU = R_NED_to_ENU * C_NED * transpose(R_NED_to_ENU)
```

不能仅交换对角线；NE、ND、ED 等相关项也必须按矩阵变换正确调整符号。转换后仍需执行现有 covariance finite/symmetric/PSD/uncertainty 检查。NAV-COV 缺失或无效时，不把 `h_acc/v_acc` 直接伪装成统计协方差；在本项目当前 fail-closed 策略下应拒绝该样本，直到单独设计并验证有来源依据的精度模型。

## 3. Header 时间戳不能替代接收单调时间或 GNSS 历元

在固定的上游 `ublox_dgnss_node.cpp` 中：

- USB 输入回调使用 `rclcpp::Clock().now()` 生成队列时间；
- `UBXNavPVT.header.stamp` 和其他 UBX 消息 header 取自该队列时间；
- 同一消息同时独立保留 receiver `iTOW`。

所以实际实现中该 header 时间来自**主机 USB 收包路径**，不能仅凭 message 注释中的 acquisition-time 描述就认定它是接收机测量时刻，也不应将它当作 steady/monotonic freshness 时钟。项目 adapter 应在本机接收回调入口记录 `std::chrono::steady_clock` 或等价 monotonic 时间，用于 freshness 与跨 topic 收包偏差；GNSS 源历元配对则只用同一 receiver 的 `iTOW`。

`iTOW` 是 GPS 周内毫秒计数，会每周回绕，也会在接收机重新启动/时间状态重建时出现跳变。禁止直接将 `iTOW * 1e6` 永久当作严格递增的 `source_timestamp_ns`。adapter 还必须建立可审计的 `source_clock_id`/receiver generation 与跨周连续时间映射：优先基于通过有效性标志确认的 GNSS week/UTC 时间字段，并结合 NAV-STATUS 的 startup 计数检测重启；不能确定时间状态时清空缓存并重启质量门控连续计数。

## 4. 不直接使用上游 NavSatFix 转换节点的原因

上游 `ublox_nav_sat_fix_hp_node` 的源码显示：

- HPPOSLLH callback 复制位置消息的 header 并发布 NavSatFix；
- 卫星状态由单独的 NAV-STATUS callback 写入成员变量；
- 位置协方差由单独的 NAV-COV callback 写入成员数组；
- 位置 callback 没有将 NAV-STATUS/NAV-COV 的 `iTOW` 与当前 HPPOSLLH `iTOW` 比较，而是使用最近一次到达后保存在成员中的状态/协方差。

因此消息到达顺序或延迟不同时，输出 NavSatFix 可能组合不同 GNSS epoch 的位置、状态和协方差。这个节点可以用于普通显示/非严格场景的参考，但**不得未经修改和历元回归测试直接接入本项目 RTK quality gate 或 global EKF**。项目以原始 UBX messages 构建自有 bounded exact-iTOW join。

此外，上游 `ublox_rover_hpposllh_navsatfix.launch.py` 示例显式配置 HPPOSLLH、NAV-STATUS、NAV-COV，但没有显式打开 NAV-PVT 输出；本项目 launch/config 必须显式启用所有需要的 UBX 消息，并核验实际固件回读值和每条消息的输出频率。上游示例的 `CFG_RATE_*` 参数不是本项目已经测定的输出频率，禁止照搬成项目验收指标。

## 5. 对比方案

### `Sustainable-Robotics-Base-for-Crops/zed_f9p_gnss`

- [仓库](https://github.com/Sustainable-Robotics-Base-for-Crops/zed_f9p_gnss)，审查的 main commit `fa1a5ec075c41d3b4d8ca34036f7ddca83932e95)，Apache-2.0。
- README 声明每次 NAV-PVT 收到后发布 GPSFix 和 NavSatFix；但 NavSatFix 消息没有 `iTOW` 字段。
- 源码在收到 UBX sync 首字节时调用 `this->now()` 填充消息 header，而不是从 GNSS measurement epoch 生成时间戳。
- 将 NAV-PVT 的解状态压缩映射到普通 NavSatStatus，无法独立表达 carrier solution Float/Fixed。
- 仓库 CI 当前配置为 Jazzy/Noble；未在本轮核验 Humble build。

结论：结构简单，但若不修改消息和时间契约，不能满足本项目严格同历元 RTK quality gate；不作为当前首选。

### 项目自有 `robodog_gnss_quality`

继续复用现有纯算法核心中的 UBX flags 解码、精确 iTOW 配对和 quality gate；未来新增 ROS 2 adapter 与bounded epoch join。上游驱动只负责传输和原始 UBX message decode，项目 adapter 负责来源可信、历元匹配、时间连续性、协方差转换及门控。上游 driver 不应直接发布到 Nav2 global localization 而绕过该门控。

## 6. 构建验证和未完成事项

本仓库 CI 使用固定 commit `9d4f5addbc72d092c57dde99b20712258e96e81e`，分别在 Humble/Jammy 与 Jazzy/Noble 容器构建上游 `ublox_dgnss` 元包，并检查 UBXNavPVT/HPPosLLH/COV 消息接口。该 CI 只证明软件源码可在对应 ROS 发行版构建；不证明 USB 设备发现、serial/USB权限、接收机固件、NTRIP 改正、RTK Fixed 输出或 PPS。

仍待完成：

- 在真实 simpleRTK2B/ZED-F9P 板卡确认 USB VID/PID、CDC-ACM 设备路径、固件版本与串口/USB 配置。
- 只在设备连接后验证 NAV-PVT/HPPOSLLH/NAV-COV/NAV-STATUS 的实际 topic、频率、丢包与 iTOW 匹配率。
- 确认 NAV-COV 有效率与协方差 ENU 变换；无效协方差样本按策略拒绝。
- 实现 ROS adapter 的有界缓存、严格同历元 join、接收 monotonic 时间戳、receiver week/boot generation 和 reset handling。
- 验证 NTRIP/基站改正链、Fixed/Float/Single、遮挡、多路径与恢复行为。
- 如需 PPS，再单独核验板卡引脚、电压、电平转换、主机 GPIO/PHC 采集路径和时间误差；RTK Fixed 不等于 PPS/PTP 同步。
- 将 qualified NavSatFix 接入 `navsat_transform_node`/global EKF 前，先完成 bag replay、状态退化和跳变测试。

**当前结论：** 已选择并固定一个更符合 iTOW 原始消息配对要求的 ROS 2 驱动候选；还没有完成项目自己的 ROS adapter、实际接收机数据、PPS、NTRIP 或 global EKF 集成，因此 C6 必须继续标记为部分完成，不可标记为 RTK 系统已可用。
