# 外部公开数据集替代路线（UrbanLoco P2 解阻塞）

- 核查日期：2026-11-24
- 状态：公开资料核验完成；尚未下载、校验、转换或回放任何替代数据
- 适用任务：Point-LIO/LIO、腿式状态估计、传感器时间同步、动态障碍物场景、Nav2 仿真前置验证
- 根进度台账：`../../TaskPlan.md`

## 1. 决策

**不因 UrbanLoco `HK-Data20190117.bag` 无法获取而更换 Lite3、MID-360、双目相机、ZED-F9P 或 AGX Orin。** 这是指定公开数据源的可获取性问题，不构成硬件选型失败证据。

将单一 UrbanLoco P2 拆成三条互补的公开数据基准：

1. **GrandTour：首选综合四足 + MID-360 基准。** ANYmal D 实际搭载 Livox MID-360、腿式状态估计/关节数据、双 RTK-GNSS 和 Leica 全站仪参考；传感器组合与目标系统最接近。
2. **Quadruped-SLAM / QuadSLAM：四足步态与本体状态基准。** Unitree Go2 + Hesai XT16，包含 IMU、关节/足端接触和 Leica TS60 参考轨迹。
3. **M3DGR：MID-360 / GNSS / 动态行人场景基准。** 多传感器地面机器人数据；有 MID-360、其 IMU、相机和 GNSS/RTK，但不是四足平台。

UrbanLoco 保留为可选历史复现；未获得来源可信的原始 ROS1 bag 或转换链前，原来的 `HK-Data20190117_0.db3` 仍只作诊断样本，不能改名冒充原始数据。

## 2. 数据集对比与适用边界

| 数据集 | 平台与传感器 | 真值/状态 | 数据规模与访问 | 项目用途 / 限制 |
|---|---|---|---|---|
| [GrandTour](https://grand-tour.leggedrobotics.com/) | ANYmal D；3 类 LiDAR（含 Livox MID-360、Hesai XT32、VLP-16）、多相机、8 个 IMU、12 关节本体测量、双 RTK-GNSS | Leica MS60 + AP20；后处理 RTK；官方提供腿式状态估计输出 | 全集约 2.11 TB；ROS bag 通过 Kleinkram 注册访问；ZARR 在 Hugging Face 免注册 | 最接近目标的“真实四足 + MID-360 + GNSS”联合数据。ANYmal 与 Lite3 不同，厂商 odometry、运动学和安装外参不可直接移植。不得下载整个数据集。 |
| [Quadruped-SLAM](https://github.com/EN3D-Lab/Quadruped-SLAM-dataset) | Unitree Go2 + Hesai XT16；IMU、关节编码器、足端接触/力、LowState/HighState | Leica TS60 全站仪跟踪 360° prism，作者称毫米级参考轨迹 | 22 条序列；ROS1 bag；Road00 示例 1,240 s、12.1 GB；Google Drive | 专项验证四足步态、振动、打滑、失衡恢复、台阶/坡地；不是 MID-360 或 RTK 数据集。 |
| [M3DGR](https://github.com/sjtuyinjie/M3DGR) | 地面机器人；Livox MID-360 + MID-360 IMU、Avia、RealSense D435i、Insta360 X4、轮式里程计、CUAV GNSS/RTK | 按场景提供 RTK、Mocap 或其他参考；每条序列需单独核对 GT 文件 | 32 条序列、约 225 GB；ROS1 bag；OneDrive / 阿里云盘；仓库提供 SHA256 清单 | 首先验证 MID-360 驱动/点云、IMU、GNSS topic 适配与动态人员/遮挡场景；平台为轮式地面机器人，不能代替四足步态验证。 |

### 2.1 GrandTour：首选综合基准

官方传感器清单明确列有 Livox MID-360（10 Hz，PTP）、ANYmal 关节执行器测量（400 Hz）、ANYmal IMU（400 Hz）、CPT7 RTK/PPP GNSS（10 Hz），并列有 Leica MS60 + AP20 地面真值。其派生状态估计使用关节与 IMU 融合，包含 odometry 与足端接触相关状态。

数据集全量超过 TB 级。官方数据页说明 ROS 格式需注册后通过 Kleinkram 获取；ZARR 格式由 Hugging Face 提供且免注册。先从任务页面选择单个代表 mission，检查单 mission 大小和可用字段；禁止把 2.11 TB 全集作为首轮下载目标。

- 官方数据集与访问格式：[GrandTour Dataset](https://grandtour.leggedrobotics.com/dataset)
- 传感器、状态估计和标定：[Setup](https://grandtour.leggedrobotics.com/setup)
- ZARR 镜像：[Hugging Face dataset](https://huggingface.co/datasets/leggedrobotics/grand_tour_dataset)
- 示例与处理代码：[grand_tour_dataset](https://github.com/leggedrobotics/grand_tour_dataset)

**许可证核对备注：** 官方网站声明数据为 CC BY-SA 4.0，软件为 MIT；Hugging Face 页面当前 metadata 显示 `mit`。这是数据集页面与镜像 metadata 的标注不一致。项目使用前以官方数据集条款为准，并记录该差异；对外再分发派生数据前应向维护者确认。

### 2.2 Quadruped-SLAM：四足运动基准

作者公开 22 条 Go2 序列，包含平地、坡道、楼梯、草地、林地、狭窄走廊和地下停车场等场景，以及趴卧、行走、爬楼、跑动、足端打滑、失衡和恢复等状态。示例 Road00 是 12.1 GB、1,240 秒、约 187 万条消息，包含：

- `/go2_imu_sensor`：约 500 Hz；
- `/low_state` 与 `/sport_mode_state`：约 500 Hz，含关节、电机、足端力、步态、机身位置/速度等字段；
- `/velodyne_points`：Hesai XT16，约 10 Hz，作者转换为 VLP-16 点格式的 PointCloud2；
- 由 Leica TS60 跟踪 prism 得到的参考轨迹。

- 数据说明与下载：[Quadruped-SLAM README](https://github.com/EN3D-Lab/Quadruped-SLAM-dataset)
- 作者公开 Google Drive 文件夹：[Road00 等数据入口](https://drive.google.com/drive/folders/1iiKPCqm6VnbNZGPadHuzbj_0v0nYyc33?usp=sharing)

仓库根目录当前未见独立 LICENSE 文件，README 也未明确说明数据许可。**在作者许可/数据使用条款确认前，只将其作为待授权研究候选；不得假定可商业使用或再分发。**

### 2.3 M3DGR：最快启动的 MID-360 对照基准

M3DGR 官方传感器清单包含 Livox MID-360（10 Hz）及其内置 IMU（200 Hz）、Avia、RealSense D435i、Insta360 X4、WHEELTEC 轮式里程计、CUAV GNSS/RTK。其 ROS topics 包括：

- `/livox/mid360/lidar`、`/livox/mid360/imu`
- `/livox/avia/lidar`、`/livox/avia/imu`
- `/camera/color/image_raw/compressed` 与深度图
- `/ublox_driver/receiver_pvt` 等 GNSS topics

推荐首批仅下载小序列，不拉取全量 225 GB：

| 优先级 | 序列 | 官方标注 | 大小 / 时长 | 官方 SHA256 |
|---|---|---|---:|---|
| 1 | `Dynamic03.bag` | 室外动态行人 | 3.20 GB / 284 s | `A54B687A924EE4E9B30673F85D88AC73615A10FCE4A7AB40F6099C24283BCC78` |
| 2 | `Outdoor01.bag` | 标准室外 | 6.10 GB / 411 s | `00A270807BCA336BC5191320B9D453CEFEF76507850286E91063BC726C3775C8` |

- 下载与序列索引：[M3DGR README §5](https://github.com/sjtuyinjie/M3DGR#5-dataset-sequences)（每条序列列出 OneDrive / 阿里云盘入口）
- 官方 SHA256 清单：[checksums.txt](https://github.com/sjtuyinjie/M3DGR/blob/main/checksums.txt)
- 传感器配置与 topic：[README §4](https://github.com/sjtuyinjie/M3DGR#4-sensor-setup)
- 备用阿里云盘分享入口（以 README 对应序列行的链接为准）：[阿里云盘分享](https://www.alipan.com/s/ee1svBzXqBz)

**许可证核对备注：** 仓库中的 MIT LICENSE 可确认其软件代码许可；当前 README 未明确给出数据文件自身的许可条款，不能据软件 MIT 许可推断所有 bag/GT 数据均可任意再分发或商业使用。必要时向作者确认。

## 3. 下载与验收顺序

### 第一批：由用户手动下载

1. **M3DGR `Dynamic03.bag`（3.20 GB）及该序列对应 GT 文件。** 用官方 `checksums.txt` 校验 bag SHA256；如果镜像中只有压缩包，先保留原压缩包并记录 SHA256，再按官方流程解包，不能用解包文件大小替代校验。
2. **Quadruped-SLAM `Road00.bag`（示例 12.1 GB）及对应参考轨迹。** 从作者 Google Drive 入口获取；该数据尚无已核实的上游 SHA256，下载后计算本地 SHA256，记为本地接收完整性证据，而不是“上游 hash 匹配”。

首轮不要求下载 M3DGR `Outdoor01`；Dynamic03 通过完整性和 topic audit 后再补充。GrandTour 不下载全量，先由用户确认能否注册访问 ROS bag 或浏览 ZARR mission，再由单 mission 大小决定下一批。

### 第二批：资料/格式门禁

每个数据集都要形成独立 manifest，至少包含：来源页面、数据许可状态、下载时间、原始文件名与字节数、SHA256（上游值/本地计算值分别记录）、ROS1/ROS2/ZARR 格式、转换工具及版本、custom message 定义版本、topic/type/count、时间跨度、参考轨迹格式与来源。

M3DGR 和 QuadSLAM 当前公开示例为 ROS1 bag。目标开发环境是 Ubuntu 24.04 / ROS 2 Jazzy；转换前必须解决 Livox / Unitree 自定义消息定义和时间戳语义。不得假定仅执行一次通用转换命令就能无损保留所有自定义消息。

### 第三批：算法回放与比较

- LIO：先验证原始 LiDAR/IMU topic 与 Point-LIO 输入要求；分别报告原始数据、转换数据和算法参数。
- 轨迹：只在 GT 时间基准、坐标系、外参和时间偏移核验后运行 `evo_ape` / `evo_rpe`；不能把导航地图坐标直接当 GT。
- 四足状态：解析 Go2/ANYmal 的 joint/contact/body state，作为运动状态分析与潜在腿式里程计对照；不得将其他机器人的腿式 odometry 直接接入 Lite3 控制。
- 动态避障：M3DGR 的 Dynamic03 只代表动态人员场景数据，不能自动视为逐帧行人检测标注。检测/跟踪指标需要人工或可验证的标注来源。
- Nav2 闭环：数据集离线回放不是闭环导航验收。动态障碍物重规划、安全停车和控制链仍需独立 Gazebo 仿真；实机测试必须另过硬件安全门。

## 4. 本项目的证据边界

- GrandTour 是当前最强的外部四足 + MID-360 组合基准，但平台为 ANYmal D，不是 Lite3 Venture。
- QuadSLAM 能评估 Go2 的步态和状态估计输入，但 LiDAR 为 Hesai XT16，不等同 MID-360。
- M3DGR 能检验 MID-360 和 GNSS 数据接口/算法，但平台为轮式地面机器人。
- 三套数据集均不能证明本项目的 See3CAM 双目、ZED-F9P 固件、Lite3 SDK、Jetson Orin 部署、Nav2 实时闭环或真实制动安全已验收。
- 当前 `HK-Data20190117_0.db3` 的已完成诊断仍有效，但数据源 provenance 未解决；不删除、不篡改、不晋升为官方 UrbanLoco 样本。

## 5. 官方来源

- GrandTour dataset / setup / Hugging Face：[dataset](https://grandtour.leggedrobotics.com/dataset)、[sensor setup](https://grandtour.leggedrobotics.com/setup)、[Hugging Face](https://huggingface.co/datasets/leggedrobotics/grand_tour_dataset)
- QuadSLAM：[GitHub README](https://github.com/EN3D-Lab/Quadruped-SLAM-dataset)
- M3DGR：[GitHub README](https://github.com/sjtuyinjie/M3DGR)、[checksums](https://github.com/sjtuyinjie/M3DGR/blob/main/checksums.txt)
