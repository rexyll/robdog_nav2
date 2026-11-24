# 机载供电与载荷预算（采购前工程核算）

- 日期：2026-11-24
- 状态：初步预算，**不是可直接接线的最终电气设计**
- 唯一进度台账：[根目录 TaskPlan.md](../../TaskPlan.md)
- 硬件决策：[ADR-0002](../decisions/ADR-0002-hardware-platform.md)

## 1. 核算边界

本预算针对以下预选组合：
- DEEP Robotics Lite3 Venture；
- Livox MID-360；
- 2 × e-con Systems See3CAM_50CUG_CXXCC_BX_H01R1 Color；
- ArduSimple simpleRTK2B Budget / ZED-F9P + ANN-MB 天线；
- NVIDIA Jetson AGX Orin Developer Kit（64GB）。

Lite3 Venture 公开标称载荷 4.5 kg。项目内部上装质量目标暂定 3.6 kg，以保留约 20% 余量；该目标是内部设计余量，不是厂商批准的上装质量或重心限制。真正可用载荷还受重心、转动惯量、跌倒冲击、支架和动态步态限制。

## 2. 已知设备质量

| 设备 | 已知质量 | 数量 | 小计 | 证据与限制 |
|---|---:|---:|---:|---|
| Jetson AGX Orin Developer Kit | 约 872.5 g | 1 | 872.5 g | NVIDIA 开发者论坛硬件 FAQ 回答；需以实际交付套件称重 |
| Livox MID-360 | 265 g | 1 | 265 g | Livox 官方规格 |
| simpleRTK2B Budget 板 | 19.5 g | 1 | 19.5 g | ArduSimple 官方规格 |
| ANN-MB 天线 | 173 g | 1 | 173 g | ArduSimple 官方 ANN-MB 规格；不含未单独列明的安装件 |
| See3CAM_50CUG Color | 未公开确认 | 2 | 待称重 | 官方页面给出外形尺寸 40 × 40 × 45.23 mm，不等于质量 |
| 双目镜头 | 待称重 | 2 | 待称重 | 需确定具体镜头 SKU |
| Ethernet 交换机 | 待称重 | 1 | 待称重 | 根据端口数、供电方式和工业温度范围选择 |
| 双目硬件触发/隔离板 | 待称重 | 1 | 待称重 | 电气规格、隔离和连接器待确认 |
| DC/DC、PD 电源板、保险和分配 | 待称重 | 1 套 | 待称重 | 取决于 Lite3 电源接口额定和 Orin 输入方案 |
| 上装板、减振件、天线支架、线束 | 待称重 | 1 套 | 待称重 | 需要 CAD、结构设计和实物称量 |

**当前已知质量小计：约 1.330 kg。** 该小计不含两台相机和所有安装/电源附件，不能视为整机最终上装质量。

对 3.6 kg 内部目标，已知质量之后剩余约 2.270 kg 供未称重项目使用。阶段 A 采购 Gate 必须通过真实 BOM 称量验证；若超目标，优先评估工业级 Orin NX 载板方案、减少外壳/支架重量或改用独立传感器电源布局，不得超过厂商载荷/重心限制。

## 3. 已知用电负载

| 设备 | 公开功率/电流 | 数量 | 估算功率 |
|---|---:|---:|---:|
| Jetson AGX Orin 64GB 模块功率预算 | 15–60 W | 1 | 15–60 W |
| Livox MID-360 | 平均 6.5 W；9–27 V DC | 1 | 6.5 W |
| See3CAM_50CUG Color | 0.44–1.225 W | 2 | 0.88–2.45 W |
| simpleRTK2B Budget | 80 mA @ 5 V | 1 | 0.40 W |
| ANN-MB 天线 LNA | 15 mA @ 3 V | 1 | 0.045 W |
| Ethernet 交换机 | 未选型 | 1 | 待确认 |
| USB/触发分配、风扇和指示灯 | 未选型 | — | 待确认 |
| DC/DC/USB-PD 转换损耗 | 依器件效率与负载 | — | 待实测 |

已知设备的理论负载合计约 **22.8–69.4 W**。这是模块/设备端的估算，不是机器人电池端的最终功率；尚未包含交换机、线损、转换效率、风扇、启动峰值和任何冗余设计。

NVIDIA AGX Orin 64GB 模块具有 15–60 W 电源预算模式。Jetson AGX Orin Developer Kit 用户指南说明 USB-C PD sink 支持 20V/4.5A（90W），NVIDIA 支持论坛也将开发套件电源适配器标为 90W。机载供电需满足开发套件输入协议与峰值，不应只按模块 60W 设计。

## 4. 推荐电源拓扑（待 Lite3 厂商确认后定型）

    Lite3 Venture 24V auxiliary rail (current rating TBD)
            |
            +---- protected 24V rail ------------------> Livox MID-360
            |
            +---- DC/DC + USB-PD source (rating TBD) ---> AGX Orin Dev Kit
            |                                                |
            |                                                +--> Left USB3 camera
            |                                                +--> Right USB3 camera
            |                                                +--> ZED-F9P (USB)
            |
            +---- regulated auxiliary rail (if needed) --> Ethernet switch / trigger board

    All grounds, chassis bonding, E-stop, fuse coordination and transient protection
    must follow the robot manufacturer's electrical requirements.

### 4.1 MID-360

- MID-360 官方输入范围为 9–27V DC，平均功率 6.5W。
- Lite3 Venture 官方示例展示了 MID-360 从 Venture 的 24V XT30 供电。
- 本项目可将机器人 24V 辅助支路作为候选，但只有在厂商书面确认端口电流、峰值、电源纹波、连接器和保险之前，不允许直接接线。
- MID-360 Ethernet 数据接入交换机/AGX 网络；需要独立确认 IP、UDP、网络负载与 PTP 架构。

### 4.2 AGX Orin Developer Kit

- Developer Kit 可通过 DC jack 或 USB-C PD 输入供电。
- NVIDIA 用户指南列出的 USB-C PD source profile 包含 20V/4.5A；这对应 90W 级供电能力。
- 如果由 Lite3 24V 辅助口供电，必须使用能从 24V 输入生成兼容 USB-C PD 的 DC/DC/PD source；不能把 24V 直接接到 AGX 输入。
- DC/DC 必须按 90W 级输入需求、启动/峰值、环境温度、效率、散热和保护选择。供应商额定和实际测试确认前，不锁定具体型号。
- 若 Lite3 24V 口无法提供所需电流，候选方案为单独的辅助电池/隔离电源；增加的质量和续航影响必须纳入预算。

### 4.3 双目相机

- 每台相机官方典型功率范围 0.44–1.225W，使用 USB 3.2 Gen1。
- AGX Orin Developer Kit 提供多个 USB 3.x host ports；每台相机尽量连接到不同高速端口/独立 USB 控制器路径（需在实机上核对 USB topology）。
- 两台相机的 Trigger 引脚必须通过合规缓冲/扇出/电平匹配板连接同一触发源；不能未经手册确认就直接并联 GPIO。
- USB host 供电和双相机并行带宽须在满分辨率/目标帧率下实测。

### 4.4 RTK

- simpleRTK2B Budget 官方典型功率约 0.40W（80mA @ 5V），ANN-MB 天线约 0.045W（15mA @ 3V）。
- 可先由 Jetson USB host 供电并经 USB/UART 接入；需要验证 USB 供电余量、地线/噪声和接收机状态消息完整性。
- 若使用板上 Timepulse/PPS 接口连接主机，要先确认引脚电压、电气输入和信号完整性；不能直接把未验证电平接入 GPIO。

## 5. 供电和结构的采购 Gate

在取得 Lite3 Venture 厂商书面答复之前，下列内容均为阻塞项：

- [ ] 24V/12V/5V 每路持续电流、峰值电流、总功率、浪涌和短路保护；
- [ ] 24V XT30 接口可否同时供 MID-360 和 DC/DC，是否需要独立保险；
- [ ] 是否允许 90W 级 USB-PD DC/DC 长时间从本体辅助电源取电；
- [ ] AGX Orin 开发套件实际重量、尺寸、输入电源与持续满载热设计；
- [ ] 全部相机、镜头、交换机、触发板、电源、安装件和线束质量；
- [ ] 质量中心/惯量/上装抗跌倒和动态步态限制；
- [ ] 采用独立辅助电池时，对机体载荷、质心和续航的影响；
- [ ] 所有电源线采用保险、反接保护、浪涌保护、固定和应力释放。

## 6. 采购前实测方案

收到硬件后先在桌面/台架上完成，不在机器狗行走时首次验证：

1. 独立测量 AGX 各功耗模式的输入电压、电流和启动峰值。
2. 两路相机同时运行 5MP 目标帧率，测量 USB 拓扑、丢帧和功耗。
3. MID-360 运行点云并开启 PTP，测量电源波动和网络带宽。
4. RTK 启用 GNSS、USB 和 PPS 后测量耗电及信号干扰。
5. 所有设备并行运行至少 30–60 分钟，记录输入功率、温度、降频和重启。
6. 单独测试断电/欠压/短路保护；不得将可能引起运动失控的电源故障测试在机器狗行走状态进行。
7. 只有确认供电和线束稳态后，才安装到机器狗上，并先静态站立监控。

## 7. 相关官方规格

- [Lite3 Venture 产品规格](https://www.deeprobotics.us/products/lite-3/)
- [Lite3 Venture 官方感知扩展示例](https://developer.deeprobotics.cn/docs/lite3/examples/sdk-dev/perception-algorithm-example/)
- [Livox MID-360 规格](https://www.livoxtech.com/mid-360/specs)
- [e-con Systems See3CAM_50CUG Color 产品规格](https://www.e-consystems.com/sony-imx264-camera.asp)
- [ArduSimple simpleRTK2B 规格](https://www.ardusimple.com/product/simplertk2b/)
- [ArduSimple simpleRTK2B 接收机与 ANN-MB 天线规格](https://www.ardusimple.com/simplertk2b-receivers/)
- [NVIDIA AGX Orin Developer Kit 用户指南 — 供电与 USB](https://docs.nvidia.com/jetson/agx-orin-devkit/user-guide/howto.html)
- [NVIDIA Jetson AGX Orin 64GB 功率模式](https://docs.nvidia.com/jetson/archives/r39.2.2/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html)
