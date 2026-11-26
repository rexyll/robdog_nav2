# UBX-NAV-TIMEGPS / UBX-NAV-TIMELS payload 解码器

日期：2026-11-26  
分支：nav  
协议基线：u-blox ZED-F9P Interface Description UBX-18010854 R06 / protocol 27.10（固定驱动源码与目标设备固件仍需分别核实）

## 1. 本轮实现

新增纯 Python 模块：
- src/robodog_gnss_quality/robodog_gnss_quality/ubx_time_messages.py
- src/robodog_gnss_quality/test/test_ubx_time_messages.py

新增 decode_nav_timegps_payload()：
- 精确 payload 长度 16 bytes；
- iTOW：U4 little-endian，限制在 [0, 604800000) ms；
- fTOW：I4 little-endian，保留 signed ns，限制在 [-500000, +500000] ns；
- week：I2 signed；
- leapS：I1 signed；
- valid：bit0 towValid、bit1 weekValid、bit2 leapSValid，原始 validity byte 与 reserved high bits 同时保留；
- tAcc：U4 ns。

新增 decode_nav_timels_payload()：
- 精确 payload 长度 24 bytes；
- iTOW：offset 0 U4；
- version：offset 4 U1，目前仅接受 0；
- reserved1：offset 5..7 原样保留；
- srcOfCurrLs：offset 8 U1，currLs：offset 9 I1 signed；
- srcOfLsChange：offset 10 U1，lsChange：offset 11 I1 signed；
- timeToLsEvent：offset 12 I4 signed；
- dateOfLsGpsWn：offset 16 U2，dateOfLsGpsDn：offset 18 U2；
- reserved2：offset 20..22 原样保留；
- valid：offset 23，bit0 validCurrLs、bit1 validTimeToLsEvent，reserved high bits 保留。

两个 decoder 都要求调用方提供 receive_monotonic_ns、process_session_id、connection_generation，并将其保存在 immutable dataclass 中，供后续 lifecycle fence 使用。解析器只检查结构、范围和字段编码；不将任何 validity bit 自动解释为可信 GPS clock/leap-second source。

## 2. 可信度边界

- NAV-TIMEGPS 是 GPS 时间交叉核验候选；towValid、weekValid、leapSValid 必须分别检查。不得因消息存在就假定 week 或 leap seconds 有效。
- NAV-TIMELS 的 validCurrLs 与 validTimeToLsEvent 是两个独立状态。future/last event validity 不能替代 current leap-second validity。
- srcOfCurrLs=0（firmware default）与 255（unknown）不得自动成为可信当前 offset；其他来源也必须经过项目 source policy 和固件/设备证据。
- parser 保留 signed currLs / lsChange；禁止把 I1 误读为 U1。
- reserved bytes / reserved validity bits 被保留，当前不因非零而静默丢弃，也不赋予未定义语义。
- parser 未执行 GPS week 展开、UTC civil conversion、leap second normalization、NAV-PVT/TIMEUTC/TIMELS exact epoch join 或 source freshness policy。

## 3. 固定来源证据

u-blox ZED-F9P Interface Description UBX-18010854 R06：
- NAV-TIMEGPS class/id 0x01/0x20，payload 16 bytes；字段布局见 §5.14.21，尤其 fTOW signed I4、week signed I2、leapS signed I1 与 validity bits。
- NAV-TIMELS class/id 0x01/0x26，payload 24 bytes；字段布局见 §5.14.22，current leap source 与 signed offset、future event fields、validity bits 分离。
- 本文档所引用 R06 是协议 27.10 的公开版本。R06 文档状态为 Early Production Information，不是目标设备固件证据。必须采集目标 ZED-F9P MON-VER（FWVER/PROTVER/MOD）并与具体 Interface Description revision 对齐后，才能冻结部署 profile。

## 4. 合成测试

测试覆盖：
- TIMEGPS signed fTOW/week/leapS、各 validity bit 独立状态、reserved bits；
- TIMELS signed currLs/lsChange/timeToLsEvent、来源 ID、reserved bytes、独立 current/future validity；
- 两种 payload 的短/长长度拒绝；
- TIMELS 未知 version 拒绝；
- iTOW/fTOW 越界；
- process session / connection generation / monotonic receive timestamp 必填和类型检查。

当前测试为 synthetic byte vectors；不能代替真实 F9P raw UBX capture、目标固件协议兼容性、USB/ROS ingress generation barrier 或可信 mapper 验收。
