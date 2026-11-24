# Point-LIO ROS 2 Jazzy POC 来源、补丁与验收基线

- 日期：2026-11-24
- 状态：Point-LIO 固定来源/P0、ROS 2 Jazzy x86_64 P1 构建已通过；UrbanLoco P2 工具与 10 项解析/ENU/SQLite integration tests 已通过；实际 rosbag 回放仍等待人工获取 CC BY-NC-SA 4.0 数据；Orin ARM64 与真实 MID-360/Lite3 实机测试未完成
- 唯一进度台账：[TaskPlan.md](../../TaskPlan.md)
- 相关基线：[MID-360 Livox driver](livox_mid360_driver_baseline.md)、[odometry fallback strategy](odometry_fallback_strategy.md)

## 1. 固定来源（不得跟随浮动分支）

| 组件 | 精确来源 | 作用 | 状态 |
|---|---|---|---|
| Point-LIO upstream | https://github.com/hku-mars/Point-LIO.git，commit \`97b0042e397ec71c327966abebff9b4b7e2c154a\` | Intel Robotics AI Suite 当前 Point-LIO 子模块 gitlink 所指向的干净上游基线 | 固定 |
| Intel reference repo | \`open-edge-platform/edge-ai-suites\` commit \`decbb06fc1b746b229957b66b92ab972e1f05609\` | 保存 Point-LIO 的 ROS 2 Jazzy 参考移植及补丁 | 固定 |
| Patch 0001 | blob \`1adc74543fd2194997fd1971178055cd64aa7636\`；format-patch commit \`8bc6098ebf007aaf04c9a4c552916694d0bd34cd\` | ROS1/catkin → ROS2/ament_cmake；Livox ROS Driver2；MID-360 配置；profiling | 固定 |
| Patch 0002 | blob \`0f308178c88f10909e71be77fb91d24f8ec00207\`；format-patch commit \`c018b76643f94a52eeffde53946d5bf41f33d6cd\` | clang-format 格式化，无算法逻辑变化 | 固定 |
| Patch 0003 | blob \`c2881ad1d715bd8a6907a141c5ef210a2599be66\`；format-patch commit \`86c04405f88ef2f892142d1c12c2e97939bc92cc\` | 修复 IMU 初始化提前退出、距离阈值拼写和若干边界/异常处理 | 固定 |
| Livox ROS Driver2 | tag \`1.2.6\`，commit \`13eb05e4e6dd7a765b934d0c5fd6236676a57b49\` | ROS 2 Jazzy 驱动 | 固定 |
| Livox SDK2 | tag \`v1.3.1\`，commit \`f5d9375f84efe2b15bc0a052d3e18482ed13adf4\` | Driver2 依赖；GCC 13 下按 Intel 参考加 \`-include cstdint\` | 固定 |

**为什么使用 gitlink 的 97b... 而不是 Point-LIO 分支当前 HEAD：** Intel 的补丁是针对其 reference repo 的子模块精确 commit 制作的。直接把较新的分支 HEAD 当基线可能导致补丁上下文偏移或重复包含上游修改。项目先复现 Intel 锁定的基线，后续升级上游时另行做补丁迁移和回归。

固定来源：
- [Intel Point-LIO reference README at fixed repo commit](https://github.com/open-edge-platform/edge-ai-suites/blob/decbb06fc1b746b229957b66b92ab972e1f05609/robotics-ai-suite/pipelines/point-lio-demo/README.md)
- [Intel fixed .gitmodules](https://github.com/open-edge-platform/edge-ai-suites/blob/decbb06fc1b746b229957b66b92ab972e1f05609/.gitmodules)
- [Point-LIO exact upstream base](https://github.com/hku-mars/Point-LIO/tree/97b0042e397ec71c327966abebff9b4b7e2c154a)
- [Patch 0001](https://github.com/open-edge-platform/edge-ai-suites/blob/decbb06fc1b746b229957b66b92ab972e1f05609/robotics-ai-suite/pipelines/point-lio-demo/patches/0001-Port-Point-LIO-to-ROS2-and-add-benchmarking-instrume.patch)
- [Patch 0002](https://github.com/open-edge-platform/edge-ai-suites/blob/decbb06fc1b746b229957b66b92ab972e1f05609/robotics-ai-suite/pipelines/point-lio-demo/patches/0002-Reformat-sources-to-match-the-project-s-real-clang-f.patch)
- [Patch 0003](https://github.com/open-edge-platform/edge-ai-suites/blob/decbb06fc1b746b229957b66b92ab972e1f05609/robotics-ai-suite/pipelines/point-lio-demo/patches/0003-Fix-early-loop-exit-and-a-distance-threshold-typo.patch)

## 2. 来源与许可证边界

- HKU-MARS Point-LIO 上游在该固定 commit 的 LICENSE 为 BSD-3-Clause；其 ROS 2 移植后的 package.xml 也声明 BSD-3-Clause。
- Intel reference repo 根 LICENSE 为 Apache-2.0；三份补丁由该仓库按固定 commit 发布。
- Livox Driver2 声明 MIT；Livox SDK2、PCL、Eigen、ROS 2 及其传递依赖仍需纳入完整 SBOM/notice 复核。
- 仅通过公开构建不等于项目已经完成商用分发合规。产品交付前，第三方清单应分别列出上游源码、补丁来源、SDK/驱动、模型/数据集（如使用）、构建二进制和修改文件；保留所有适用 copyright/license/NOTICE，并由项目法务/合规流程复核再分发方式。

## 3. 受控准备脚本

项目脚本：[scripts/prepare_point_lio_intel_ros2.sh](../../scripts/prepare_point_lio_intel_ros2.sh)

脚本行为：
1. 在指定 workspace 的 \`src/point_lio\` clone Point-LIO；
2. checkout 并验证精确 upstream base commit \`97b0042...\`；
3. 从 Intel fixed repo commit \`decbb06...\` 下载 3 个固定补丁；
4. 按 0001 → 0002 → 0003 顺序运行 \`git am --keep-cr\`；
5. 将固定来源标记写入 Point-LIO 仓库本地 \`.git/robodog-intel-patches.lock\`；
6. 已有补丁锁标记且匹配时幂等退出；基线不符、工作区 dirty 或 lock 不符时拒绝继续，避免覆盖用户修改。

执行例：
\`\`\`bash
bash scripts/prepare_point_lio_intel_ros2.sh ~/workspace/robodog_nav2_ws
\`\`\`

脚本只准备固定源码，不安装系统依赖、不 build、不下载数据集，也不启动机器人或发送运动命令。

## 4. MID-360 专用参数：禁止照抄参考外参

Intel patch 添加 \`config/mid360_ros2.yaml\`，参考参数使用：
- \`/livox/lidar\`、\`/livox/imu\`；
- Livox custom point type 输入；
- \`lidar_type: 1\`、\`scan_rate: 10\`、\`timestamp_unit: 2\`；
- sample \`extrinsic_T: [-0.011, -0.02329, 0.04412]\` 及 identity rotation。

这些值只是该参考配置，不是本项目机器狗安装值。特别是 sample LiDAR-IMU 平移外参不得直接复制。必须通过 MID-360 安装测量/标定确定：
- \`lidar_link ↔ imu_link\` 平移和旋转；
- timestamp_unit/timebase/offset_time 的实际单位；
- LiDAR/IMU 时间 offset、PTP 状态和 drift；
- IMU 输出率、量程、轴向、饱和阈值、噪声参数；
- 每点时间有效率与点云去畸变结果。

在上述项目验收前，不能将参考 YAML 作为本项目生产参数，也不能将 Point-LIO 输出接入 Nav2 实机运动链。

### 4.1 Livox CustomMsg 逐点时间语义核验

固定版本 Livox ROS Driver2 1.2.6 的 `src/lddc.cpp`：
- `pkg.base_time` 被写入 `CustomMsg.timebase`；
- 同一值被写入 ROS 2 `header.stamp`，使用 `rclcpp::Time(timestamp)`；
- 每个 `CustomPoint.offset_time` 由 SDK 点时间减去 `pkg.base_time` 得出。

`CustomMsg.msg` 将 `offset_time` 定义为相对 base time 的时间偏移；Driver2 的 ROS 2 header stamp 和 base time 使用纳秒时钟域。Point-LIO 固定 upstream 的 Livox CustomMsg 处理函数将 `offset_time / 1,000,000` 转换为内部毫秒（Point-LIO point curvature），用于逐点时间排序/去畸变。因此当前源码链条所需语义为：
- `timebase/header.stamp`：基准采样时间，ns；
- `offset_time`：逐点相对偏移，ns；
- Point-LIO 内部 per-point time：`offset_time_ns / 1e6`，ms。

**重要：** Intel `mid360_ros2.yaml` 中的 `preprocess.timestamp_unit: 2` 不是 Livox CustomMsg 的 offset_time 换算开关。固定 Point-LIO 源码只在标准 PointCloud2 preprocess 分支按 `timestamp_unit` 设置 `time_unit_scale`；Livox CustomMsg 分支使用独立的 `offset_time / 1e6` 硬编码换算。项目不能依靠改变该参数来修正 CustomMsg 时间单位。

源码定位：
- [Livox Driver2 v1.2.6 CustomMsg.msg](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/msg/CustomMsg.msg)
- [Livox Driver2 v1.2.6 CustomPoint.msg](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/msg/CustomPoint.msg)
- [Livox Driver2 v1.2.6 lddc.cpp (CustomMsg and ImuData timestamps)](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/src/lddc.cpp)
- [Point-LIO upstream preprocess.cpp pinned base](https://github.com/hku-mars/Point-LIO/blob/97b0042e397ec71c327966abebff9b4b7e2c154a/src/preprocess.cpp)
- [Intel Point-LIO ROS2 port patch](https://github.com/open-edge-platform/edge-ai-suites/blob/decbb06fc1b746b229957b66b92ab972e1f05609/robotics-ai-suite/pipelines/point-lio-demo/patches/0001-Port-Point-LIO-to-ROS2-and-add-benchmarking-instrume.patch)

Livox Driver2 v1.2.6 的 `InitImuMsg()` 同样将 `ImuData.time_stamp` 写入 IMU `header.stamp`，ROS 2 分支也用 `rclcpp::Time(timestamp)`。因此 driver source 中 LiDAR CustomMsg 的 `timebase/header.stamp` 与 IMU header 都以 ns 作为 ROS 时间输入单位；这提供了两路时间字段的可比较基础，但**不证明** MID-360 当前 PTP 模式已启用、设备时钟已对齐到主机、或实机 LiDAR/IMU offset 为零。

该源码核验不替代设备/主机时钟同步测试。实际 PTP offset/drift、LiDAR IMU 同时钟性、CustomMsg header stamp 单调性、offset_time 范围和丢点率仍须在真实 MID-360 bag 上测量。

## 5. 分层验收，避免把 Intel benchmark 误作实机证明

| 层级 | 测试 | 能证明什么 | 不能证明什么 |
|---|---|---|---|
| P0 | 检查上游 base SHA、patch source commit、patch 顺序、dirty-tree 拒绝和脚本幂等 | 源码准备可复现 | 编译成功或算法正确 |
| P1 | Ubuntu 24.04 / ROS 2 Jazzy x86_64 build | ROS2 移植与依赖在 x86_64 可编译 | ARM64 / Orin 可编译 |
| P2 | UrbanLoco \`ulhk_4\` bag replay + 独立 GT + evo APE/RPE | 在该历史 Velodyne 数据集上作算法回归 | MID-360 per-point time、四足振动、Jetson 性能 |
| P3 | Jetson AGX Orin aarch64 build + 资源测量 | Orin 工具链和依赖可用、CPU/RAM/热约束初步合格 | 实时 MID-360 时间同步和轨迹精度 |
| P4 | 真实 MID-360 静止/手持/装机受控采集、PTP/timebase/offset_time 检查 | 实际传感器时间链、消息与外参可审计 | 四足走动中连续定位可靠 |
| P5 | Lite3 抬起/静止/受控低速步态，独立真值对比 | 真实四足运动条件下的 APE/RPE、延迟、资源和失败行为 | 未覆盖地面/速度/载荷条件的普遍保证 |

Intel 公共 UrbanLoco 示例使用 2019 Velodyne 记录，该数据缺少每点 time 字段，Intel patch 有按扫描几何估算点时间的 fallback。此 fallback 仅用于兼容旧数据集，不允许作为实时 MID-360 数据输入的常规策略。实时 MID-360 POC 必须保留真实每点时间字段。

Intel README 所述 UrbanLoco \`ulhk_4\` 论文基线 RMSE 2.17m 和 20% 回归容差只用于该公开数据集回归，不能转为本项目机器狗导航准入门槛。Intel 参考性能平台为 Intel 平台，而不是 Jetson AGX Orin。

## 6. P1 构建脚本与工作流

项目已新增：
- [scripts/build_point_lio_jazzy.sh](../../scripts/build_point_lio_jazzy.sh)：校验 Point-LIO 锁定的 upstream/Intel patch series；固定 Livox Driver2 v1.2.6 和 SDK2 v1.3.1；SDK2 用 GCC 13 -include cstdint 参数构建安装；调用官方 Driver2 ./build.sh jazzy，在同一 workspace 中构建 Driver2 与 Point-LIO；最后核对两个 ROS package prefix 和 Point-LIO executable。
- [Point-LIO ROS 2 Jazzy Build](../../.github/workflows/point-lio-jazzy-build.yml)：使用官方预装镜像 `ros:jazzy-ros-base-noble`（Ubuntu 24.04），显式安装 native PCL/Eigen、colcon、rosdep、ament 依赖，先准备固定源码，再运行上述构建脚本；不再每次从裸 Noble 镜像安装整个 ROS distro。

### 6.1 P1 编译根因与修复

Point-LIO P1 run [45335424924](https://github.com/rexyll/robdog_nav2/actions/runs/45335424924) 已成功完成 ROS Jazzy 安装、原生 PCL/Eigen 依赖安装和固定源码/补丁准备；失败发生在 build step 的 Livox SDK2 编译。

日志显示同一个 P1 script 先从 workspace/src/Livox-SDK2 独立构建并安装 SDK2，使用 -include cstdint 成功；随后执行 Driver2 官方 ./build.sh jazzy，该脚本在整个 src/ workspace 中运行 colcon。Colcon 又将 src/Livox-SDK2 自动识别为 livox_sdk2 CMake package 并二次编译，但这次未继承 GCC 13 的 -include cstdint，因此在 SDK2 define.h 对 std::uint8_t/std::uint16_t 的使用处报错，Driver2 随后被 abort。

**修复：**
- 最终将 SDK2 源码 clone 到 `${WORKSPACE}_external/Livox-SDK2`，即整个 colcon workspace 根目录之外；仅放在 workspace/vendor 仍会被默认 colcon package discovery 递归发现；
- 保留 SDK2 独立构建并安装到 /usr/local；
- 在 colcon 前检查 /usr/local/lib/liblivox_lidar_sdk_shared.so 与 /usr/local/include/livox_lidar_api.h；
- 执行 `colcon list` 并显式要求仅发现 `livox_ros_driver2`、`point_lio` 目标包，若仍发现 `livox_sdk2` 则拒绝继续。

修复脚本 commits：e9601959（最初的 vendor source path）、26fa6597（安装产物检查）、c34975e5（最终将 SDK2 source 移到 workspace 外部并加入 colcon list gate）。Point-LIO P1 Jazzy x86_64 build 已通过：[run 45337200320](https://github.com/rexyll/robdog_nav2/actions/runs/45337200320)。运行顺序完成固定来源准备、SDK2 v1.3.1 独立 build/install、Driver2 v1.2.6、Point-LIO 两个 colcon packages build/install，以及 ROS package prefix/executable 验证；日志显示 Finished livox_ros_driver2、Finished point_lio、Summary: 2 packages finished，并发现 pointlio_mapping executable。P0 source preparation run 45336709339 也成功。Livox 独立构建 [run 45337301420](https://github.com/rexyll/robdog_nav2/actions/runs/45337301420) 同样成功，Driver2 prefix/executable 检查通过。

两条 build workflow 均已增加同分支 concurrency/cancel-in-progress，并将 docs-only push/PR 从触发路径移除，以避免无效重复编译。P1 pass 仅证明 Ubuntu 24.04/Jazzy x86_64 编译/安装，不证明 Orin ARM64 或实时传感器性能。

### 6.2 P3a：原生 ARM64 Ubuntu/Jazzy 构建预验证

新增 [Point-LIO ROS 2 Jazzy ARM64 Build workflow](../../.github/workflows/point-lio-jazzy-arm64-build.yml)，使用原生 ubuntu-24.04-arm runner 和多架构 ros:jazzy-ros-base-noble ARM64 容器：
- workflow 会断言 uname -m=aarch64 与 dpkg --print-architecture=arm64；
- 同一固定源码准备脚本和 build_point_lio_jazzy.sh 构建 SDK2/Driver2/Point-LIO；
- 构建后检查 SDK2 shared library、ROS package prefixes 和 pointlio_mapping executable。

ARM64 run [45339955465](https://github.com/rexyll/robdog_nav2/actions/runs/45339955465) 与 [45340700005](https://github.com/rexyll/robdog_nav2/actions/runs/45340700005) 的实际日志均显示 SDK2/Driver2/Point-LIO 编译和安装已成功；前两轮仅分别被后置环境检查的 `file` 命令缺失、独立 shell 未 source ROS 环境所阻断。修复后第三轮 [run 45341347087](https://github.com/rexyll/robdog_nav2/actions/runs/45341347087) **最终 success**：日志确认 ARM64 原生构建环境、colcon 仅发现 `livox_ros_driver2` 与 `point_lio` 两个包，二者均 Finished，汇总 `2 packages finished`；SDK2 构建/安装步骤成功；source ROS Jazzy 与 workspace `install/setup.bash` 后，两个 ROS package prefix、`pointlio_mapping` 可执行文件以及 `ros2 pkg executables` 后置检查全部通过。日志存在 PCL/CMake policy 与 Boost placeholder 开发警告，但不影响退出状态和检查结果。

GitHub 文档说明公开仓库可使用 ubuntu-24.04-arm 原生 ARM64 runner（4 vCPU/16 GB）；ROS Jazzy Noble base image 提供 linux/arm64 multi-platform variant。该 gate 即使通过，也只证明通用 Ubuntu 24.04/Jazzy aarch64 构建，不等价于 Jetson AGX Orin / JetPack 7.2.2 构建，更不测 CUDA/TensorRT、Jetson 内核、GPIO/网卡硬件时间戳或实际热/功耗。正式 P3 仍需在目标 Orin 设备上复测并采集 CPU/RAM/温度/功耗。

## 7. P2 UrbanLoco u l h k_4 离线回放工具

P2 目标：使用 Point-LIO 固定 ROS 2 Jazzy port，对 UrbanLoco 香港序列 \`ulhk_4\` / \`HK-Data20190117\` 进行**Velodyne 历史数据集兼容性回归**，把 Point-LIO odometry 与 NovAtel SPAN-CPT INSPVAX ground truth 统一为 TUM 轨迹，使用 evo APE 进行 Umeyama SE(3) 对齐。该数据集不是 MID-360 数据，也不验证 Livox per-point time。

### 7.1 数据许可和人工准备

- 官方数据许可为 CC BY-NC-SA 4.0，非商业、署名、相同方式共享条件；任何商业使用/再分发必须先确认权利人授权。
- 本 POC 仅选定 Intel Point-LIO `ulhk_4` 对应的 UrbanLoco session `HK-Data20190117`，不是同目录里其他几十 GB 的香港序列。UrbanLoco 官方 Hong Kong 页面标注该序列日期为 2019/01/17、总量 6.11 GB；其他序列分别可达 24.0/27.9/41.6/62.3 GB，不可替代。UrbanLoco GitHub README 该条目的日期列误写为 2019/03/16，因此以序列 ID 为准。
- Dropbox/Baidu 链接是包含全部香港序列的共享目录，而不是单文件直链；应在目录内找到 `HK-Data20190117` 条目。若看不到该条目，不要改下其他大文件，先提供当前目录名称/大小清单确认。Intel Point-LIO 复现文档把下载后的 ROS 1 文件命名/放置为 `datasets/ulhk_4/HK-Data20190117.bag`；若镜像提供压缩包或不同名称，先检查实际格式与内容，不要仅通过改名伪装成 `.bag`。
- 原始文件放置：\`datasets/ulhk_4/HK-Data20190117.bag\`；脚本转换到 ROS 2 sqlite3 bag：\`datasets/ulhk_4/ulhk_bag/\`。
- 下载镜像见 [UrbanLoco dataset preparation instructions](../../datasets/README.md)。整个 \`datasets/*\` 已加入 .gitignore，不得提交 raw bag、转换后的 db3、metadata、轨迹和 PCD/map。
- 已在 Intel 固定参考脚本中核对该序列的消息 topic：LiDAR \`/velodyne_points_0\`、IMU \`/imu/data\`、GT \`/novatel_data/inspvax\`。运行时仍由 \`ros2 bag info\` 检查，若实际 bag 不同必须以数据自身 metadata 为准，不静默替换。

### 7.2 INSPVAX CDR 字段偏移的来源核验

本项目 GT extractor 的 CDR byte offsets 不是通过自造 fixture 猜测：
- NovAtel 官方 ROS message definition 固定到 commit `0c0e135b71c5920b845982358f6c7ec71a952b5e`，`INSPVAX.msg` 依次声明 `latitude`、`longitude`、`height` 为 float64：[官方 INSPVAX.msg](https://github.com/novatel/novatel_oem7_driver/blob/0c0e135b71c5920b845982358f6c7ec71a952b5e/src/novatel_oem7_msgs/msg/INSPVAX.msg)。
- Intel Point-LIO reference repo 固定 commit `decbb06fc1b746b229957b66b92ab972e1f05609` 的 `extract_ulhk_gt.py` 明确使用 little-endian doubles，offsets 为 latitude=60、longitude=68、height=76：[Intel 固定提取脚本](https://github.com/open-edge-platform/edge-ai-suites/blob/decbb06fc1b746b229957b66b92ab972e1f05609/robotics-ai-suite/pipelines/point-lio-demo/scripts/extract_ulhk_gt.py)。
- 本项目的 `urbanloco_gt_helpers.py` 和 SQLite rosbag integration tests 使用相同的固定 offsets，并增加 payload 长度、有限值与地理范围检查。

**限制：** 该偏移映射仅适用于 UrbanLoco HK-Data20190117 中匹配的 `novatel_oem7_msgs/msg/INSPVAX` 序列化布局。实际 bag 下载后必须再次核对 SQLite topics.type、CDR payload 大小、首尾及抽样解码合理性；若 bag topic type 或 message schema 不一致，extractor 必须 fail closed，不可按相同 offset 强行输出 GT。

### 7.3 项目脚本

1. scripts/prepare_urbanloco_ulhk4.sh：检查手动下载的 ROS1 bag，调用 rosbags-convert 生成 ROS2 sqlite3 bag，并校验三个 topic；每次运行重算 raw bag SHA256、生成 DB3 checksum 清单和 dataset_manifest.txt（采集日期可通过 URBANLOCO_ACQUISITION_DATE 指定）。
2. \`scripts/run_urbanloco_ulhk4.sh\`：加载已通过 P1 的 \`point_lio\` 安装空间，启动 \`pointlio_mapping\` 与 TUM recorder，确认 LiDAR subscription 和 odometry pub/sub ready 后，以 1x 播放 bag；完成后清理进程并检查有效轨迹样本数量。
3. \`scripts/extract_urbanloco_ulhk4_gt.py\`：从 rosbag2 sqlite3 的 INSPVAX CDR payload 提取固定 UrbanLoco session 中的 lat/lon/height 字段，按 bag timestamp 排序、去重并转换为 WGS84 ECEF→local ENU，输出 TUM + JSON provenance report。固定 byte offsets 只对该 session/type layout 有效，换数据版本前必须重新验证。
4. \`scripts/evaluate_urbanloco_ulhk4.sh\`：运行 \`evo_ape tum GT EST -a\`，将 2.17m（Point-LIO 论文该序列参考值）与 +20% 区间作为历史参考输出；它不是本项目导航验收阈值，不能用来判定 MID-360/Orin/Lite3 通过。
5. \`scripts/tests/test_urbanloco_ulhk4.py\`：对 CDR payload 范围检查、地理字段有效性、ENU 方向/高度和 bag timestamp 排序去重做纯 Python 回归。
6. \`.github/workflows/point-lio-urbanloco-p2-tests.yml\`：CI 运行 py_compile、bash -n、CLI help 和测试，无需下载或提交 UrbanLoco 数据。

### 7.4 当前证据与下一步

- [x] P2 第一版脚本/CLI 静态检查和 8 项解析/ENU/排序测试通过：run 45339580363。
- [x] 增加 raw bag / DB3 SHA256 provenance、SQLite rosbag INSPVAX GT extraction integration tests 与 replay process-group cleanup 后，P2 语法/CLI/纯函数/SQLite integration 共 10 项测试通过：[run 45340287093](https://github.com/rexyll/robdog_nav2/actions/runs/45340287093)。
- [x] 将 NovAtel 官方 INSPVAX.msg（固定 commit 0c0e135b...）与 Intel 固定 POC 的 `extract_ulhk_gt.py`（固定 commit decbb06...）列为 CDR offsets 的上游依据；source-link/注释更新后再次运行 [P2 CI 45341097202](https://github.com/rexyll/robdog_nav2/actions/runs/45341097202)，10/10 tests passed。
- [x] P2 数据目录许可/忽略策略已写入 \`datasets/README.md\` 与 .gitignore。
- [ ] 人工下载并放置 UrbanLoco \`HK-Data20190117.bag\`，记录来源、许可、文件大小与 SHA256。
- [ ] 执行 ROS1→ROS2 转换，检查 bag metadata、消息数、时间跨度和预期三个 topic。
- [ ] 完成 Point-LIO 全量 bag playback，保存 TUM estimate、logs、manifest。
- [ ] 从 INSPVAX 提取 GT，核验 CDR offsets/时间基准，并完成 evo APE/RPE 与结果报告。
- [ ] 对结果做敏感性检查：起始时段、时间覆盖率、重复/倒退时间戳、有效轨迹比例；不得只依赖单一 RMSE。

**边界：** UrbanLoco 是 Velodyne 历史数据，因此即使 P2 全部通过，也只说明该公开数据集上的算法回归有效，不代表 MID-360 CustomMsg 每点时间、PTP/IMU 同步、Jetson Orin 性能或四足运动场景通过。

## 8. 当前状态

- [x] 上游子模块 commit、Intel reference commit 和三份 patch blob/commit 均已固定。
- [x] Point-LIO upstream BSD-3-Clause、Intel patch repository Apache-2.0 的来源边界已记录。
- [x] 固定源码准备脚本已创建并经 P0 GitHub Actions 实测成功。
- [x] 运行准备脚本并确认三个补丁均成功应用（GitHub Actions P0 run 45334159646；base SHA 97b0042...；three patches applied; worktree clean）。
- [x] 在 Jazzy x86_64 构建 Point-LIO（run 45337200320，2 个 ROS packages finished）。
- [~] P2 回放/GT/evo 工具静态检查、纯函数和 SQLite INSPVAX GT extraction integration 共 10 项测试已通过（runs 45340287093、45341097202）；等待人工获取 UrbanLoco 数据后进行实际 bag 回放和独立真值评估。
- [x] 通用原生 ARM64 Ubuntu 24.04/Jazzy 构建预验证通过：[run 45341347087](https://github.com/rexyll/robdog_nav2/actions/runs/45341347087) success；SDK2、Livox ROS Driver2、Point-LIO 编译/安装成功，两个 ROS package prefix、`pointlio_mapping` 与 `ros2 pkg executables` 后置检查全部通过。仅证明通用 aarch64 构建链可用，不代替 Jetson AGX Orin/JetPack 7.2.2 实机 build、CUDA/TensorRT、Jetson 内核、GPIO/硬件时间戳或 CPU/RAM/温度/功耗测试。
- [ ] 连接真实 MID-360，确认 CustomMsg 每点时间、IMU/PTP 时间一致性。
- [ ] 安装至 Lite3 后进行受控运动 POC 和真值评测。

**阶段判断：** 已达到固定来源、可复现 x86_64 软件构建和 P2 工具验证基线；未达到实际 UrbanLoco bag 回放、Orin ARM64、MID-360 时间链或 Lite3 四足实机定位验收。
