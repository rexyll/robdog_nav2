# Point-LIO ROS 2 Jazzy POC 来源、补丁与验收基线

- 日期：2026-11-24
- 状态：上游与补丁来源已固定；准备脚本已提交；尚未完成本项目 Point-LIO 编译、UrbanLoco 回放或 Orin 实机测试
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

## 6. 当前状态

- [x] 上游子模块 commit、Intel reference commit 和三份 patch blob/commit 均已固定。
- [x] Point-LIO upstream BSD-3-Clause、Intel patch repository Apache-2.0 的来源边界已记录。
- [x] 固定源码准备脚本已创建。
- [ ] 运行准备脚本并确认三个补丁均成功应用。
- [ ] 在 Jazzy x86_64 构建 Point-LIO。
- [ ] 用公开 UrbanLoco bag 完成可复现离线回放和独立真值评估。
- [ ] 在 Orin aarch64 构建并测量 CPU/RAM/热。
- [ ] 连接真实 MID-360，确认 CustomMsg 每点时间、IMU/PTP 时间一致性。
- [ ] 安装至 Lite3 后进行受控运动 POC 和真值评测。

**阶段判断：** 该方案已达到可复现源代码准备基线，但尚未达到项目软件构建通过，更未达到四足实机定位验收。
