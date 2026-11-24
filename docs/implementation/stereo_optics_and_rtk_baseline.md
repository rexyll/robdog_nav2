# 双目光学、硬件同步与 RTK 接入基线

- 日期：2026-11-24
- 状态：公开规格核验与初步工程参数已完成；镜头、触发电气和 RTK 改正服务仍需供应商/现场确认
- 唯一进度台账：[根目录 TaskPlan.md](../../TaskPlan.md)

## 1. 双目相机选型复核

项目候选相机保持为 **2 × e-con Systems See3CAM_50CUG_CXXCC_BX_H01R1 Color**（无镜头）：

- Sony IMX264，5MP，2448 × 2048，2/3 英寸，3.45 μm 像元；
- Global Shutter，官方页面标称 5MP@35fps；
- USB 3.2 Gen 1、Type-C，彩色输出 YUY2；
- 6-pin GPIO header 提供 I2C、Strobe 和 Trigger；
- C-Mount 镜头座；相机本体典型功耗 0.44–1.225W；
- 官方页面确认外部触发能力，但没有给出足以设计触发分配板的完整 GPIO 电压、极性、最小脉宽、输入电流和触发延迟参数。

资料：[e-con Systems See3CAM_50CUG Color 产品页](https://www.e-consystems.com/sony-imx264-camera.asp)。

## 2. 镜头候选与视场计算

### 2.1 初选候选

将 **Commonlands CIL530 6mm C-Mount** 列为送样/适配验证候选，不立即冻结采购：

| 参数 | 候选规格 |
|---|---:|
| 焦距 | 6mm |
| 光圈 | F1.8 |
| 镜头格式 | 最高 2/3 英寸 |
| 名义分辨率 | 5MP / 3.3 μm |
| 成像圆 | 11mm |
| 标称畸变 | -2.0% |
| 质量 | 167g/只 |
| IR-cut | 页面标示无内置 IR-cut filter |

资料：[Commonlands CIL530 6mm 镜头](https://commonlands.com/products/6mm-c-mount-lens-twothirdsinch-cil530)。

按相机有效分辨率 2448 × 2048、像元 3.45 μm 计算，传感器有效尺寸约 8.446 × 7.066mm，对角线约 11.0mm。6mm 镜头按针孔模型估算水平视场约 **70.2°**、垂直视场约 **61.0°**。

注意：镜头 11mm 成像圆与传感器约 11mm 对角线接近，边缘照度/暗角必须实测；其 IR-cut 配置、滤镜安装方式和 IMX264 全画幅实际 MTF 也要向供应商确认。若有暗角、色偏或边缘分辨率不足，不得只凭标称 5MP 规格采购。

### 2.2 双目基线与深度灵敏度

- 初始 CAD 目标：双目基线 **120mm**，两光轴尽量平行；安装板保留俯仰角调节，先在约 0°至向下 10°范围内做视场验证，不在此阶段锁死角度。
- 两相机同型号、同镜头、同焦距/光圈设置；刚性安装，标定后防松。
- 以 6mm 焦距、3.45 μm 像元、120mm 基线估算，像素焦距约 1,739px。理想针孔模型下，若视差误差为 0.25px，深度误差量级约为：3m 处 1.1cm、5m 处 3.0cm、10m 处 12cm。
- 上述是误差灵敏度估算，不是实际精度保证；纹理、曝光、标定残差、畸变、步态振动、时间偏差和遮挡都会使实际误差增大。Stereo 只作为 MID-360 的补充几何源，不承担唯一安全停车职责。

## 3. 双相机 USB 带宽与触发

彩色 YUY2 按每像素 2 bytes 估算，单路 2448 × 2048 约 10.03MB/frame：

- 20fps：约 1.60Gbit/s/相机，两路合计约 3.21Gbit/s（仅图像有效载荷估算）；
- 35fps：约 2.80Gbit/s/相机，两路合计约 5.61Gbit/s。

因此初始实测配置建议 **5MP、20fps、YUY2**；35fps 只在验证 Jetson USB root-hub 拓扑、控制器共享带宽、协议开销和持续无丢帧后再尝试。不能因为开发板有多个 USB 插口就假设它们拥有独立带宽。

触发设计：
1. 以共同硬件触发源扇出到左右相机；触发分配板需具备与相机 GPIO 电气参数匹配的缓冲/电平转换和适当隔离。
2. 未取得 pinout、电压、极性、脉宽和输入电流参数前，不可把两个相机 GPIO 直接并联，也不可将 Jetson GPIO 直接接入相机触发脚。
3. 向 e-con 索取彩色型号专用 datasheet、GPIO pinout、硬件触发时序图、Linux/Jetson SDK 控制方式，以及 5MP YUY2 连续流模式下是否支持外触发。
4. 用示波器/逻辑分析仪和图像采集时间戳验证两路曝光起始偏差、触发丢失、重复帧和 USB 丢帧；设置曝光/增益一致，验证运动边缘重影。
5. ROS 输出应提供左右独立 image topic、各自 CameraInfo、统一 optical frame 命名和采集时间；完成标定后再发布 stereo disparity/depth/PointCloud2。

## 4. RTK 接入基线

候选保持为 **ArduSimple simpleRTK2B Budget（u-blox ZED-F9P）+ 双频多星座天线**：

- 当前产品页列出的板卡 SKU 为 AS-RTK2B-F9P-L1L2-NH-03；板卡基于 ZED-F9P，可连接 Nvidia Jetson，支持 Base/Rover，高性能配置下最高 10Hz RTK 定位输出。
- 板卡本身不含天线；必须另配 L1/L2 多频 GNSS 天线。ANN-MB 是当前质量预算中的候选（公开标称 173g），最终应按安装尺寸、馈线和机器人顶部结构确认。
- RTK 固定解需要外部改正：NTRIP Caster、区域 RTK 网络，或自建基站/基准站；没有改正流时不能宣称厘米级定位。项目作业地点尚未确定，暂不指定 NTRIP 服务商。
- 初始 ROS 接入目标为 10Hz；记录 UBX fix type、carrier solution（FIX/FLOAT）、差分改正龄期、卫星数、位置精度估计和时间戳，经过质量门控后再输入 NavSatFix / navsat_transform_node。
- 单天线 GNSS 在静止时不能可靠提供机身 yaw；航向由经验证的 IMU/腿式 odometry 连续估计，或后续另评估双天线定向方案。
- 天线优先布置在顶部并保证开阔天空视野；需实测多路径、馈线布局以及 DC/DC、USB3、Jetson 和高速 Ethernet 线束引入的电磁干扰。

资料：
- [simpleRTK2B Budget 产品和规格](https://www.ardusimple.com/product/simplertk2b/)
- [simpleRTK2B Budget 用户指南](https://www.ardusimple.com/user-guide/simplertk2b-budget/)
- [ArduSimple 4G/NTRIP Kit（NTRIP client/server 用途参考）](https://www.ardusimple.com/product/simplertk2b-v3-4g-ntrip-starter-kit/)

## 5. 下一轮验收项目

- [ ] 取得 See3CAM 彩色版本 GPIO 电气/触发时序资料，并确认 Jetson 双 UVC 同时采集能力。
- [ ] 采购前取得 CIL530 样品或同等规格镜头，验证全画幅暗角、MTF、畸变、IR-cut、最小工作距离和锁紧结构。
- [ ] 用 120mm 基线、5MP@20fps 录制运动场景，输出双目标定重投影误差、左右曝光偏差、深度误差与无效视差比例。
- [ ] 确认 RTK 天线 SKU、安装位置、馈线长度、NTRIP/基站来源和目标场地覆盖。
- [ ] 在静态开阔天空、树荫/建筑遮挡、多路径、断改正和 FIX/FLOAT 切换条件下测量 RTK 质量、恢复时间及定位跳变。
- [ ] 所有外设称重和功耗测试完成前，不把上装预算或供电拓扑标记为采购批准。
