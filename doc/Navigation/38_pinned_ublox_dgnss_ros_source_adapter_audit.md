# 固定版 ublox_dgnss ROS 输入源适配审计

- 审计对象：`aussierobots/ublox_dgnss`
- 固定提交：`9d4f5addbc72d092c57dde99b20712258e96e81e`
- 审计日期：2026-11-26
- 范围：NAV-PVT、NAV-HPPOSLLH、NAV-COV、NAV-STATUS 的 ROSIDL 字段、publisher 转换、时间戳来源与现有 NavSatFix helper。
- 结论：该版本能提供四路所需的 decoded UBX ROS topics，但不提供原始完整 flags byte，也不携带 process session / connection generation / GPS week。可作为 decoded source；不能未经额外 generation instrumentation 就作为 reconnect-safe source。

## 1. 四路上游 topic 与 ROSIDL

固定版 node 创建以下 publisher，topic 使用相对名称：

| ROS topic | ROSIDL | 所需字段 | 上游字段情况 |
|---|---|---|---|
| `ubx_nav_pvt` | `UBXNavPVT` | `itow`, `gnss_fix_ok`, `carr_soln.status`, `invalid_llh`, `gps_fix.fix_type` | 全部存在 |
| `ubx_nav_hp_pos_llh` | `UBXNavHPPosLLH` | `itow`, `lon/lat/height`, HP residuals, individual invalid flags | 全部存在 |
| `ubx_nav_cov` | `UBXNavCov` | `itow`, `pos_cor_valid`, six NED upper-triangle terms | 全部存在 |
| `ubx_nav_status` | `UBXNavStatus` | `itow`, `gps_fix_ok`, `carr_soln_valid`, `carr_soln.status` | 全部存在 |

固定版文件证据：

- `ublox_ubx_msgs/msg/UBXNavPVT.msg`
- `ublox_ubx_msgs/msg/UBXNavHPPosLLH.msg`
- `ublox_ubx_msgs/msg/UBXNavCov.msg`
- `ublox_ubx_msgs/msg/UBXNavStatus.msg`
- `ublox_dgnss_node/src/ublox_dgnss_node.cpp` 中四路 publisher 与 message population 函数。

上游 ROSIDL 暴露 decoded semantic fields，而不是完整原始 UBX flags byte：
- NAV-PVT：`gnss_fix_ok`、`diff_soln`、`carr_soln.status`、`invalid_llh`；
- HPPOSLLH：每个 LLH/HP 分量分别有 `invalid_*` bool，不应压缩成单个 bit0；
- NAV-COV：`pos_cor_valid` / `vel_cor_valid` bool；
- NAV-STATUS：`gps_fix_ok`、`carr_soln_valid`、`carr_soln.status` 等 decoded fields。

因此本项目 `GnssNav*` normalized messages 与 source adapters 使用 decoded semantics；不得将其描述成 raw flags preservation。若未来要求原始字节可审计，必须从 UBX Frame ingress 另行保留 payload/raw flag byte。

## 2. 时间戳合同

上游 `ublox_in_callback()` 每次 USB callback 开始时调用：

```cpp
rclcpp::Time ts = rclcpp::Clock().now();
```

然后将该 `ts` 放入 `ubx_queue_frame_t`，后续四路 publisher 将 `f->ts` 赋给各自 ROS message 的 `header.stamp`。

因此该 Header stamp 是主机 USB 接收 callback 的时钟采样，不是接收机 GPST measurement epoch，也不是 GNSS week/TOW。它可以用于主机侧粗略到达时间观测，但不能替代：
- UBX `iTOW`；
- 由 NAV-TIMEGPS + NAV-TIMELS 验证的 GPST `source_timestamp_ns`；
- 在同一主机 steady clock 域采样的 callback `receive_monotonic_ns`。

本项目 adapter 显式忽略 upstream Header stamp，并在接收回调处使用 host monotonic clock。GPS week 必须来自有效 receiver-time mapping，不得从 iTOW 单独推断。

## 3. 字段转换与安全约束

### NAV-PVT
- `gnss_fix_ok` 直接映射；
- `carr_soln.status`: 0=no solution, 1=float, 2=fixed；未知编码 fail-closed/UNKNOWN；
- `invalid_llh=true` 时拒绝该观测 epoch；
- NAV-PVT 与 NAV-STATUS 的 fix validity / carrier solution 必须交叉一致。

### NAV-HPPOSLLH
- `lon/lat`: (1e-7) degree units；
- `lon_hp/lat_hp`: (1e-9) degree residual units；
- `height`: mm，`height_hp`: 0.1 mm；
- 每个 `invalid_lon/lat/height/lon_hp/lat_hp/height_hp` 独立校验；不得把 HPPOSLLH 的 8 个 flags 压缩成一个 invalidLlh bit。
- hMSL 与其 HP residual、hAcc/vAcc 被 normalized ROSIDL 保留，但当前 `GnssObservation` 只消费 ellipsoid height 与位置坐标；accuracy 字段尚未进入 quality gate。

### NAV-COV
- `pos_cor_valid` 必须为 true，否则不生成 observation；
- 六个 NED covariance terms 必须有限；
- (C_{ENU} = R C_{NED} R^T)，转换结果 row-major：
  `[EE, NE, -ED, NE, NN, -ND, -ED, -ND, DD]`。
- 与固定版 `ublox_nav_sat_fix_hp_node` 的矩阵转换公式一致。

### NAV-STATUS
- `gps_fix_ok` 与 NAV-PVT 必须一致；
- `carr_soln_valid` 必须在 fixed carrier solution 时为 true；
- `carr_soln.status` 从 `flags2.carrSoln` 解码，不是 `fixStat` 中的 `carrSolnValid` 位。不要把它们混为同一个字段。

## 4. 上游 NavSatFix helper 不作为本项目权威来源

固定版 `ublox_nav_sat_fix_hp_node`：
- HPPOSLLH callback 使用最近一次缓存的 covariance 和 status；
- NAV-COV callback 更新缓存 covariance，但没有在 callback 中以 `pos_cor_valid` gate；
- NAV-STATUS callback 更新缓存 NavSatStatus；
- HPPOSLLH callback 发布 NavSatFix 时没有将三个 topic 的 `itow` 做 exact match；
- 输出 NavSatFix 丢失原始 UBX iTOW 与接收机 generation 信息。

这意味着在三个 ROS topic 到达顺序/频率不同的情况下，NavSatFix 可能组合不同历元的 position/covariance/status。本项目必须使用独立的 bounded exact-iTOW joiner，不把该 `/fix` 输出当作历元一致性或 RTK quality 的权威证明。

## 5. 仍缺失的 generation/lifecycle metadata

上游四路 ROSIDL 均没有：
- `process_session_id`；
- `connection_generation`；
- GPS week；
- USB device attach/detach generation；
- 可与 NAV-TIMEGPS/TIMELS pairing 对齐的 receiver-time source clock identity。

本项目 normalized messages 要求上述上下文。当前 field-level adapter functions 可以将上游 decoded ROSIDL 转换成内部 immutable records，但需要调用方显式提供这些 context。它们不能自行猜测 reconnect generation。

有效 receiver-time mapping 可提供 GPS week、process session 和 connection generation，并通过 exact iTOW 关联；但这本身不能证明某条旧上游 callback 一定属于当前 USB generation，因为上游消息不带 generation。要实现严格 reconnect fence，仍需要以下之一：
1. 在上游 driver 内增加 connection generation / lifecycle event，并在每条 NAV message 中透传；
2. 由拥有设备连接的 transport bridge 发布同一 generation context，且确认不会与 ublox_dgnss 同时打开同一 USB device；
3. 以明确的 driver restart/device reconnect lifecycle 作为新的 session/generation fence。

在 generation source 未落实前，normalized topic pipeline 是已测试的软件合同，不是对上游 driver reconnect safety 的完整集成。

## 6. 配置与运行验证缺口

四类消息输出依赖对应 CFG_MSGOUT 参数启用：
- `CFG_MSGOUT_UBX_NAV_PVT_USB`
- `CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB`
- `CFG_MSGOUT_UBX_NAV_COV_USB`
- `CFG_MSGOUT_UBX_NAV_STATUS_USB`

必须在目标设备读取实际配置并验证四路输出频率、iTOW 对齐、丢帧与 reconnect 行为。NAV-COV 的目标 ZED-F9P firmware/profile 可用性也必须用实际设备和对应 Interface Description 核实；仅凭驱动中存在 parser/publisher 不代表目标固件会输出该消息。

当前没有目标设备 MON-VER、CFG readback、真实 ROS topic capture、四流 iTOW 对齐统计或 USB reconnect 实测证据。

## 7. 本项目实现与验收边界

本项目已新增：
- `GnssNavPvt`、`GnssNavHpPosLlh`、`GnssNavCov`、`GnssNavStatus` normalized ROSIDL；
- `GnssObservation` output contract；
- 与固定版上游 ROSIDL 字段对齐的 field-level source adapter functions；
- generation-fenced `UbxRosEpochRuntime`；
- exact-iTOW + validated receiver-time mapping joiner node。

最终测试通过之前的实现/文档状态不得视为上游 driver integration。Humble/Jazzy synthetic ROS CI 只验证 normalized message contract 与 join algorithm。真实上游订阅、generation lifecycle source、设备配置、ZED-F9P 物理运行与定位质量验收均保持 OPEN。

## 8. 固定源码链接

- [UBXNavPVT.msg](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_ubx_msgs/msg/UBXNavPVT.msg)
- [UBXNavHPPosLLH.msg](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_ubx_msgs/msg/UBXNavHPPosLLH.msg)
- [UBXNavCov.msg](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_ubx_msgs/msg/UBXNavCov.msg)
- [UBXNavStatus.msg](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_ubx_msgs/msg/UBXNavStatus.msg)
- [ublox_dgnss_node.cpp](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_dgnss_node/src/ublox_dgnss_node.cpp)
- [ublox_nav_sat_fix_hp_node.cpp](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_nav_sat_fix_hp_node/src/ublox_nav_sat_fix_hp_node.cpp)
