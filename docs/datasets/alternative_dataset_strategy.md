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

## 6. M3DGR Dynamic03 首次获取记录（2026-11-24）

### 6.1 已完成：原始 bag 上游完整性验证

用户在 Ubuntu 24.04 本地执行 `scripts/verify_public_dataset_checksum.sh`，得到：

- 文件：`~/Downloads/Dynamic03.bag`
- 大小：3,434,689,371 bytes（约 3.20 GB）
- 官方 SHA256：`A54B687A924EE4E9B30673F85D88AC73615A10FCE4A7AB40F6099C24283BCC78`
- 本地 SHA256：`a54b687a924ee4e9b30673f85d88ac73615a10fce4a7ab40f6099c24283bcc78`
- 结果：大小写归一后完全匹配；**上游文件完整性 Gate 通过**。

这只证明本地文件与 M3DGR 官方 checksums 清单登记的 Dynamic03 字节内容一致；尚未证明 ROS1 bag 可完整读取、话题/类型/自定义消息可解码、时间基准正确或 GT 配对正确。

### 6.2 必须补充下载：Dynamic03 独立 GT 文件

M3DGR 官方 README 的 Dynamic03 行同时提供 rosbag 与独立 GT 两组下载地址。请下载与 `Dynamic03` 同名/同序列的 GT，不要使用 Dynamic04 或其他序列的 GT：

- GT OneDrive：[Dynamic03 GT](https://1drv.ms/t/c/2b4bfc0edf421186/EYYRQt8O_EsggCsWEAAAAAAB-G7sz7ob3JbPDr8K1eYEDg?e=8GwPON)
- GT 阿里云盘：[Dynamic03 GT](https://www.alipan.com/s/2GQnM2gePFr)
- 原始 bag 下载入口（仅供复查）：[Dynamic03 rosbag OneDrive](https://1drv.ms/u/c/2b4bfc0edf421186/EYYRQt8O_EsggCsDEAAAAAABSFVtB101b-_ZORJTrA3yaw?e=K62Aoy) / [阿里云盘](https://www.alipan.com/s/ee1svBzXqBz)

GT 文件当前尚未下载和解析。取得后先记录文件名、字节数、SHA256、文本行数/字段列数、时间戳范围、重复/倒退和坐标分量，不要直接运行 evo。

### 6.3 Dynamic03 传感器 topic 与 Point-LIO 配置来源

M3DGR 官方 README 列明 MID-360 topic：

- LiDAR：`/livox/mid360/lidar`
- 内置 IMU：`/livox/mid360/imu`
- 数据集还包含 Avia、D435i 相机/IMU、GNSS 和 wheel odometry topic；首轮 MID-360 LIO 只选用 MID-360 LiDAR + 它自己的内置 IMU。

上游 M3DGR Point-LIO baseline 发布了两份容易混淆的 MID-360 配置：

- `mid360.yaml` + `mapping_mid360.launch`：LiDAR topic 为 `/livox/mid360/lidar`，IMU topic 为 `/livox/mid360/imu`，使用 MID-360 自带 IMU。
- `m3dgr_mid360.yaml` + `mapping_m3dgr_mid360.launch`：IMU topic 改为 `/camera/imu`，属于相机 IMU 路径；**本项目当前首轮 MID-360 内置 IMU 验证不要误用这份配置**。

固定上游参考：
- [M3DGR Point-LIO README](https://github.com/sjtuyinjie/M3DGR/blob/main/baseline_systems/Point_LIO_M3DGR/README.md)
- [MID-360 原生 IMU 配置 mid360.yaml](https://github.com/sjtuyinjie/M3DGR/blob/main/baseline_systems/Point_LIO_M3DGR/src/point_lio/config/mid360.yaml)
- [M3DGR 专用 camera-IMU 配置 m3dgr_mid360.yaml](https://github.com/sjtuyinjie/M3DGR/blob/main/baseline_systems/Point_LIO_M3DGR/src/point_lio/config/m3dgr_mid360.yaml)
- [M3DGR 传感器标定](https://github.com/sjtuyinjie/M3DGR/blob/main/calibration.md)

M3DGR 标定文件给出的 MID-360 LiDAR→内置 IMU 平移为 `[-0.011, -0.02329, 0.04412] m`，旋转为 identity。该标定是 M3DGR 原采集平台参数，**仅用于解释数据集配置，禁止移植到 Lite3 / 自购 MID-360 的最终 URDF/TF**。最终硬件外参必须实测。

M3DGR Point-LIO baseline 是 ROS1/catkin 代码。本项目采用固定的 ROS2 Jazzy Point-LIO port；仅把上游配置/消息语义作为适配参考，不要直接复制 ROS1 launch 或把 M3DGR 的 ROS1 baseline 当成我们的 Jazzy 构建通过证据。

### 6.4 只读结构审计工具

新增 `scripts/audit_m3dgr_ros1_bag.py`，使用纯 Python `rosbags==0.11.5` 的 `AnyReader` 读取 ROS1 bag。它会：

- 列出 connection 的 topic、ROS2-normalized msgtype、digest 和消息定义长度（若 reader 能提供）；
- 统计每个 topic 的消息数、bag timestamp 范围、平均频率、重复/倒退时间戳、raw message size；
- 只抽样解码少量 MID-360 `CustomMsg` 和 IMU，不全量反序列化点云；
- 对 MID-360 LiDAR 记录 header stamp、frame_id、timebase 原始值、声明 point_num、实际 points 数量以及 offset_time 原始 min/max；
- 对 IMU 记录 header stamp 和 bag timestamp，供之后进行时钟差核查；
- 输出 JSON，发现必需 topic/type 缺失或样本解码失败时 fail closed。

脚本不会转换或重写 bag，不推断时钟同步精度，也不会在尚未确认 message definition / driver semantics 前把 `offset_time` 数值硬解释为某个单位。

本地 Ubuntu 24.04 / Jazzy 执行：

```bash
cd ~/workspace/robdog_nav2
python3 -m venv .venv-dataset-tools
source .venv-dataset-tools/bin/activate
python -m pip install -r requirements-dataset-tools.txt
python scripts/audit_m3dgr_ros1_bag.py \\
  "$HOME/Downloads/Dynamic03.bag" \\
  --json-out datasets/m3dgr_dynamic03_audit.json
```

输出报告建议保存在被 `.gitignore` 忽略的数据目录；不要提交原始 bag 或生成的大型报告。将终端输出中的 topic/type/样本摘要及 JSON 报告关键段提供给项目审查后，再决定是否转换。


### 6.5 Livox ROS1 消息的逐点时间单位核验（固定上游来源）

本轮已将 M3DGR 仓库 `main` 固定为 commit `5fc892c84defef42feb2418f7d38897bef40f490` 后核对其自带 ROS1 Livox Driver：

- [CustomPoint.msg](https://github.com/sjtuyinjie/M3DGR/blob/5fc892c84defef42feb2418f7d38897bef40f490/baseline_systems/Point_LIO_M3DGR/src/livox_ros_driver/msg/CustomPoint.msg)：`uint32 offset_time`，描述为相对 base time 的偏移。
- [lddc.cpp：FillPointsToCustomMsg / PublishCustomPointcloud](https://github.com/sjtuyinjie/M3DGR/blob/5fc892c84defef42feb2418f7d38897bef40f490/baseline_systems/Point_LIO_M3DGR/src/livox_ros_driver/livox_ros_driver/lddc.cpp#L356-L461)：`packet_offset_time` 明确注释为 `uint:ns`；逐点 offset 以 packet offset 加 point interval 生成；每帧第一包的 packet timestamp 同时写入 `CustomMsg.timebase`，`header.stamp` 设置为 `ros::Time(timestamp / 1e9)`。
- [CustomMsg.msg](https://github.com/sjtuyinjie/M3DGR/blob/5fc892c84defef42feb2418f7d38897bef40f490/baseline_systems/Point_LIO_M3DGR/src/livox_ros_driver/msg/CustomMsg.msg)：`timebase` 注释为 first point time。

因此本数据集所对应的 ROS1 Driver 源码语义为：

- `timebase`：ns，首个 packet 时间；
- `header.stamp`：同一 packet 时间转换成 ROS 秒/纳秒；
- `CustomPoint.offset_time`：相对 `timebase` 的 ns；
- 点时刻可按 `timebase + offset_time` 计算（ns）。

新增 bag audit 会报告 `header_timebase_delta_ns`、`offset_time_raw_min/max`（单位标记 ns）、`point_num_declared` 与实际反序列化 points 数量。**仍不可从这些字段推断设备时钟同步模式**：Driver 的时间转换源码中存在 PPS/PTP/NoSync 等输入时戳分支；录制的 CustomMsg 自身并未携带足以独立确认当前 packet timestamp_type 的完整证据。需结合 bag 内 IMU 时间比较、采集配置/日志以及实际传感器测试分析；header/timebase 一致不能被表述为 PTP 已同步。

### 6.6 Dynamic03 GT 文件格式审计

M3DGR 作者的评测说明使用 TUM trajectory 格式输入 `evo_ape`；仓库中公开的 GT 示例（例如 `Grass01_fixed.txt`）为八列：`timestamp tx ty tz qx qy qz qw`。但 Dynamic03 的独立 GT 尚未由用户下载，因此不能预先断言该文件没有额外头部、是否同样为 TUM 或是否完整覆盖 284 秒。

新增 [`scripts/audit_tum_trajectory.py`](../../scripts/audit_tum_trajectory.py)，下载 GT 后先运行。审计 JSON 除 timestamp 重复/倒退数量外，还报告 `pose_repetition.adjacent_identical_pose_pairs`、`duplicate_timestamp_same_pose_pairs`、`backwards_timestamp_same_pose_pairs` 和 `timestamp_anomaly_pose_change_pairs`，用于区分时间戳异常是否发生在完全相同位姿之间；该诊断只描述数据，不进行排序或去重：

```bash
cd ~/workspace/robdog_nav2
source .venv-dataset-tools/bin/activate
python scripts/audit_tum_trajectory.py \\
  "$HOME/Downloads/Dynamic03_GT.txt" \\
  --json-out datasets/m3dgr_dynamic03_gt_audit.json
```

将 `Dynamic03_GT.txt` 替换成实际文件名。该工具只检查 TUM 八列语法、NaN/Inf、重复/倒退 timestamp、四元数范数、位置边界、时间跨度和 SHA256；不自动排序，不修复、不重采样、不对齐，也不推断时间单位/时钟或坐标系。若官方 GT 实际有头部/其他格式，工具将 fail closed；先审查真实格式，再新增显式解析器，不要删除数据行或盲目转换。

TUM audit 通过也不等于与 bag 时间对齐。下一步必须比较 GT timestamp（TUM 秒）和 bag ROS timestamp（ns 除以 1e9）的时间覆盖与起点差；仅在确认同一时钟域、时间偏移/漂移和空间参考框架后才能进行 evo APE/RPE。若不能证明时钟对齐，先按官方采集说明/作者资料补充证据，不允许用 `evo --t_offset` 试到 RMSE 最小后把它报告为真实精度。

### 6.7 ROS1 Livox 类型到本项目 ROS2 Point-LIO 的兼容性门禁

已核对两端的固定来源：

- M3DGR 配套 ROS1 Driver 的 package 为 `livox_ros_driver`，其 [CustomMsg.msg](https://github.com/sjtuyinjie/M3DGR/blob/5fc892c84defef42feb2418f7d38897bef40f490/baseline_systems/Point_LIO_M3DGR/src/livox_ros_driver/msg/CustomMsg.msg) 和 [CustomPoint.msg](https://github.com/sjtuyinjie/M3DGR/blob/5fc892c84defef42feb2418f7d38897bef40f490/baseline_systems/Point_LIO_M3DGR/src/livox_ros_driver/msg/CustomPoint.msg) 定义在旧包名下。
- 本项目 Point-LIO 固定 Intel ROS2 patch 将订阅类型从 ROS1 `livox_ros_driver::CustomMsg` 改为 ROS2 `livox_ros_driver2::msg::CustomMsg`，并把 package dependency 改为 `livox_ros_driver2`：[固定 patch 0001](https://github.com/open-edge-platform/edge-ai-suites/blob/decbb06fc1b746b229957b66b92ab972e1f05609/robotics-ai-suite/pipelines/point-lio-demo/patches/0001-Port-Point-LIO-to-ROS2-and-add-benchmarking-instrume.patch#L1311-L1314)；[消息回调类型变更](https://github.com/open-edge-platform/edge-ai-suites/blob/decbb06fc1b746b229957b66b92ab972e1f05609/robotics-ai-suite/pipelines/point-lio-demo/patches/0001-Port-Point-LIO-to-ROS2-and-add-benchmarking-instrume.patch#L2077-L2078)。

两种消息的字段布局可以相同，但 ROS 2 的 topic type 名称并不相同。因此转换前置 Gate 不是简单检查字段数量，而是检查转换输出实际注册的 ROS2 type 是否为 `livox_ros_driver2/msg/CustomMsg`，并且 ROS2 playback 中该 topic 能被 Point-LIO 的订阅类型匹配。若通用 `rosbags-convert` 保留源包名，转换结果仍会是 `livox_ros_driver/msg/CustomMsg`，Point-LIO 不会因字段布局相同就自动订阅。

**真实 Dynamic03 bag 审计（2026-11-24）补充：** 用户提供的原始 ROS1 bag 中，`/livox/mid360/lidar` 连接类型实际为 `livox_ros_driver2/msg/CustomMsg`，而不是上游旧 baseline 的 `livox_ros_driver/msg/CustomMsg`；`/livox/avia/lidar` 才是旧包名类型。两者在本次 audit 中的 digest 相同（`e4d6829bdfe657cb6c21a746c86b21a6`），message definition 长度分别为 1570/1569 bytes；真实 MID-360 抽样消息成功解码且 point_num 匹配。这说明真实输入的包名已经与目标 Jazzy Driver2 类型相同，但 ROS1 bag 内仍是 ROS1 序列化，仍需转写 ROS2 CDR rosbag2；不能因此跳过转换，也不能把实际源类型写成旧驱动名。转换器已增加对 `livox_ros_driver2/msg/CustomMsg` 与 ROS1-normalized alias `livox_ros_driver2/CustomMsg` 的白名单，并在 manifest 记录真实 source type。

转换实现应采用显式且测试过的路径，将源 Livox `CustomMsg` 与嵌套 `CustomPoint` 反序列化后按字段重建为 Jazzy `livox_ros_driver2/msg/CustomMsg`；对点坐标、reflectivity/tag/line、timebase、逐点 offset、header timestamp、point_num 做逐字段等价测试。禁止通过仅重命名 topic、伪造 metadata 或直接把 bag 文件改名来规避类型/序列化差异。

在转换器和合成 ROS1 fixture 的转换/回读测试通过之前，不要用 M3DGR Dynamic03 启动本项目 Point-LIO，更不要将回放结果接入 Nav2 控制链。

### 6.8 已实现：受控 ROS1→ROS2 LIO 输入转换器

新增 [`scripts/convert_m3dgr_dynamic03_to_ros2.py`](../../scripts/convert_m3dgr_dynamic03_to_ros2.py)，不使用盲目的通用 `rosbags-convert` 处理 Livox 类型名，而是将已审计的两个原始 topic 反序列化后逐字段重建为本项目 Point-LIO 接受的 Jazzy 消息类型：

| Topic | 已支持的 ROS1 bag 源类型 | ROS2 输出类型 |
|---|---|---|
| `/livox/mid360/lidar` | `livox_ros_driver/msg/CustomMsg`；`livox_ros_driver2/msg/CustomMsg`（本次真实 Dynamic03）；兼容 normalized alias | `livox_ros_driver2/msg/CustomMsg` |
| `/livox/mid360/imu` | `sensor_msgs/msg/Imu` | `sensor_msgs/msg/Imu` |

注意：真实 Dynamic03 的 MID-360 connection 已使用 Driver2 命名空间，所以此路是 ROS1 序列化到 ROS2 CDR 的重写，源/目标类型名相同；旧 baseline 命名空间输入仍走字段映射。

转换器的安全门禁和行为：

1. 重新计算原始 bag SHA256，并要求与仓库锁定的 M3DGR Dynamic03 官方 SHA256 完全一致；
2. 必须提供同一文件的 bag audit JSON，状态为 `structural_inputs_present`、路径/文件大小一致、topic/type/样本解码通过；
3. 对 Livox 抽样样本再次检查 `point_num == len(points)`；
4. 允许旧驱动 `uint64 ns → double seconds → ROS Time` 头时间舍入产生的最大 **1,000 ns（1 μs）** `header.stamp - timebase` 差值；超过则拒绝转换。该容差不是设备同步精度，也不是 LiDAR/IMU offset 门限；
5. 只输出两个 MID-360 LIO topic，保留每条消息的原始 bag timestamp；Livox 的 `timebase`、`point_num`、`lidar_id`、`rsvd`、每个点的坐标/reflectivity/tag/line/offset_time 均逐字段映射；IMU header、orientation、covariance、angular velocity 和 linear acceleration 逐字段复制；
6. 输出写入目标目录旁的临时 staging 目录，完整关闭 rosbag2 writer 并写入 `conversion_manifest.json` 后才原子移动为最终目录；目标已存在时拒绝覆盖；异常时清理 staging；不更改原始 ROS1 bag。

用户本地执行顺序（必须先完成 bag audit）：

```bash
cd ~/workspace/robdog_nav2
source .venv-dataset-tools/bin/activate

# 先对已下载的原始 ROS1 bag 做只读审计；exit code 0 才可继续
python scripts/audit_m3dgr_ros1_bag.py \\
  "$HOME/Downloads/Dynamic03.bag" \\
  --json-out datasets/m3dgr_dynamic03_audit.json

# 仅在审计 JSON status=structural_inputs_present 时执行
python scripts/convert_m3dgr_dynamic03_to_ros2.py \\
  "$HOME/Downloads/Dynamic03.bag" \\
  datasets/m3dgr_dynamic03_audit.json \\
  datasets/m3dgr_dynamic03_lio_ros2
```

预期输出为 ignored 的 `datasets/m3dgr_dynamic03_lio_ros2/`，内含 rosbag2 `metadata.yaml`、SQLite storage 和 `conversion_manifest.json`。转换完成后先运行 `ros2 bag info datasets/m3dgr_dynamic03_lio_ros2`，核对：

- `/livox/mid360/lidar` type 必须为 `livox_ros_driver2/msg/CustomMsg`；
- `/livox/mid360/imu` type 必须为 `sensor_msgs/msg/Imu`；
- 两路消息数应与 conversion manifest 一致；
- bag timestamp 范围、持续时间、消息频率与原始审计报告一致；
- 转换目录中的 `conversion_manifest.json` 记录源 bag SHA256、audit path、源/目标类型、消息数和时间范围。

**本次 CI 证据：** [run 45418493538](https://github.com/rexyll/robdog_nav2/actions/runs/45418493538) success：ROS1 bag 审计 5/5、TUM GT 审计 7/7、ROS1→ROS2 显式类型转换合成回读 4/4、UrbanLoco 原有回归 13/13。转换测试确认输出类型名为 `livox_ros_driver2/msg/CustomMsg`，点字段/offset/timebase 和 IMU 数据在写入 ROS2 bag 后可再读回；同时确认拒绝失败 audit、已有输出目录和大于 1 μs 的 header/timebase 差值。

**尚未完成：** 以上是合成 fixture 的转换/回读证据，不是用户真实 Dynamic03 bag 的转换结果；还未执行本地 bag audit、真实数据转换、`ros2 bag info/play`、Point-LIO replay、GT 时间/坐标关联或 evo APE/RPE。Dynamic03 独立 GT 仍需用户下载；在真实 bag/GT 审查和时钟/坐标核验完成前，禁止启动 Point-LIO 评测或宣称定位精度通过。


### 6.9 真实 Dynamic03 bag 与独立 GT 审计结果（2026-11-24）

用户本地对原始 bag 执行只读 audit，结果为 `structural_inputs_present`、`issues=[]`：

- 原始文件 3,434,689,371 bytes、210,956 条消息、284.538269992 s；官方 SHA256 已在先前步骤验证通过。
- `/livox/mid360/lidar`：2,846 条，约 10 Hz，类型 `livox_ros_driver2/msg/CustomMsg`；抽样 3/3 成功，声明 point_num 与解码点数相同，offset_time 范围约 100 ms；header/timebase 抽样差为 -77 至 +139 ns。
- `/livox/mid360/imu`：56,903 条，约 200 Hz，类型 `sensor_msgs/msg/Imu`；抽样 3/3 成功。
- 两个必需 topic 均无重复或倒退的 bag timestamp。该结果仅证明结构和抽样解码，不证明设备 PTP/PPS、IMU-LiDAR 同步、GT 正确性或 Point-LIO 精度。

独立 GT 已提交到仓库 `temp/Dynamic03.txt`（360,420 bytes，4,269 行）。逐行检查显示全部为 8 列有限 TUM 数值，四元数范数为 1 且所有四元数均为 identity；但文件包含 1,675 个重复 timestamp 和 728 个倒退 timestamp。所有 2,403 个时间戳异常都发生在相邻两行位姿完全相同的情况下，整份文件按文件顺序共有 2,847 个相邻相同位姿对；新版 TUM audit 将在 `pose_repetition` 字段中输出这一关联，便于在本地生成可复现 JSON 报告。按每连续三行检查可见 1,423 组三行位姿完全相同，组起始时间单调递增、间隔中位数约 0.2 s；这只能作为异常模式描述，不能证明三行就是冗余副本或授权删除两行。

M3DGR 官方公开的 `Grass01_fixed.txt` 也包含连续相同位姿记录且四元数保持 identity，但其时间戳仍严格递增；因此不能用该样例为 Dynamic03 的重复/倒退时间戳开脱。官方 README 说明传感器无外部触发、采用软件同步。Dynamic03 GT 时间覆盖约 284.420190 s，落在 bag 起止时间范围内；时间范围接近不证明同一时钟域或同步精度。

**当前 GT Gate：阻塞。** `scripts/audit_tum_trajectory.py temp/Dynamic03.txt` 预期返回 `blocked_review_required`（重复/倒退 timestamp），退出码 10。不得对源文件原位排序、去重、重采样或修改四元数；不得直接将该文件输入 evo。需要先核实原始下载文件是否完整、作者的 TF→TUM 导出方式及重复行语义，再决定是否生成带来源记录的派生候选轨迹。即使时间戳问题解决，仍须确认时间偏移/漂移、坐标系和 frame convention，才可运行 APE/RPE。


### 6.10 Dynamic03 GT 的独立候选生成（不替换原件）

M3DGR README 提供的 TF→TUM 工具 [Trajectory_saving_for_SLAM](https://github.com/Zjj587/Trajectory_saving_for_SLAM) 的实现采用 20 Hz 循环，并通过 `lookupTransform(..., ros::Time(0), ...)` 查询最新 TF，再写入 `transform.stamp_`。这种采样方式能够解释“同一位姿可能连续被写入多行”的现象；但 README 没有证明 Dynamic03 独立 GT 文件确由该工具生成，不能把它当作 Dynamic03 三重记录/时间乱序的正式根因，也不能据此直接修改原始文件。

新增 `scripts/create_tum_pose_run_candidate.py`，只生成派生候选目录，不修改输入文件：

```bash
cd ~/workspace/robdog_nav2
git pull --ff-only origin main
source .venv-dataset-tools/bin/activate

python scripts/create_tum_pose_run_candidate.py \\
  temp/Dynamic03.txt \\
  datasets/m3dgr_dynamic03_gt_pose_run_candidate

python scripts/audit_tum_trajectory.py \\
  datasets/m3dgr_dynamic03_gt_pose_run_candidate/trajectory.tum \\
  --json-out datasets/m3dgr_dynamic03_gt_candidate_audit.json
```

生成器的严格规则：
1. 只把文件中**连续且 tx..qw 七个 pose 字段完全相等**的记录视为同一个 pose run，不跨越不同 pose 合并。
2. 仅在该 run 内按 timestamp 排序。
3. 仅在同一个 pose run 内删除重复 timestamp（该 run 的 pose 字段完全一致，因此它是完全相同 timestamp+pose 记录）。
4. 下一 run 的最早 timestamp 必须严格大于上一 run 的最晚 timestamp；不同 pose run 只要时间范围重叠、倒退或边界 timestamp 相等就 fail closed。
5. 输出目录必须不存在；生成 `trajectory.tum` 和 `manifest.json`，manifest 记录输入/输出 SHA256、行数、排序/去重方法和限制。临时目录完整写好后才原子发布；输入 SHA256 在解析前后复核。

按仓库当前 `temp/Dynamic03.txt` 统计，候选预期为 2,594 行、1,422 个连续 pose runs（1,421 个三行 run、1 个六行 run），在 pose run 内排序并删除 1,675 条重复 timestamp+同 pose 行；原始数据中的 728 个倒退对都发生在相同 pose run 内。此数字是生成规则的可复现预期，不代表候选已由用户生成或授权为 GT。

候选文件必须保持 `ordered_tum_candidate_not_validated_ground_truth` 状态。即便第二次 TUM audit 得到 `valid_tum_trajectory`，也只证明格式和时间顺序通过：仍须确认独立 GT 的原始来源/导出过程、设备时钟域/时间偏移/漂移以及坐标系/参考 frame，之后才可将其用于 evo。若脚本在不同 pose run 边界失败，不得放宽 Gate 或跨 pose 自动排序。
