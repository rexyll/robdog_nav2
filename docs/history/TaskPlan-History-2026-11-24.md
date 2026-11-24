# RobDog Nav2 TaskPlan 变更历史归档（2026-11-24）

- 归档日期：2026-11-24
- 范围：TaskPlan v0.1 至 v0.3.4 的历史变更记录
- 当前唯一进度真源仍为根目录 [TaskPlan.md](../../TaskPlan.md)。本文件仅保存历史变更，不维护当前任务状态。

| 日期 | 版本 | 变更 | 提交/证据 |
|---|---|---|---|
| 2026-11-24 | v0.1 | 新仓库初始化、建立系统架构和 A–G 初版任务计划 | 仓库首次初始化提交 |
| 2026-11-24 | v0.2 | 预选 Lite3 Venture、MID-360、双 See3CAM_50CUG、ZED-F9P RTK、AGX Orin；建立硬件 ADR/接口清单 | ADR-0002 和接口清单提交 |
| 2026-11-24 | v0.2.1 | TaskPlan 超过 30 KB，按规则拆为架构硬件分册和测试风险分册；保留本根文件唯一进度状态 | root: 7532fc78; Part 1: 8cfc024c; Part 2: 7caa7e7d |
| 2026-11-24 | v0.2.2 | 依据官方 Lite3 ROS 2 transfer/SDK 公开接口建立 bridge contract；补充参考资料和 package 规划；A8/A9 更新为部分完成 | bridge: 83544a98; refs: 5384dd90; architecture: 8d4de365; package plan: 8fb71753; README: 5c444c50 |
| 2026-11-24 | v0.2.3 | 锁定 See3CAM 彩色无镜头 SKU；补充 ROS Lyrical 与 JetPack 7.2.2 兼容性策略：Jazzy 作为 Orin/Ubuntu 24.04 当前目标，Lyrical 等 Ubuntu 26.04/Jetson 支持成熟后评估 | e-con Systems 官方相机 SKU/规格、Open Robotics Lyrical release、NVIDIA JetPack 7.2.2/Jetson Orin 支持资料 |
| 2026-11-24 | v0.2.4 | 完成硬件 SKU/ROS 发行版策略在 ADR、接口清单、bridge、参考资料和 README 的同步；更新根 TaskPlan 进度与供应商待确认项 | Part 1: 1c7eea37; ADR-0002: 7d41f8a6; checklist: dfe25143; Lite3 bridge: feef5cf9; references: 1416b088; README: 3ab67049 |
| 2026-11-24 | v0.2.5 | 建立 Lite3 ROS 2 只读 mock/诊断软件包骨架和 Humble/Jazzy CI；补充多传感器时间同步与外参标定设计；CI 当前待运行结论 | package: 74a05789–bea3fbfd; pytest layout: ba171479–123d32ec; sync doc: a04b50dd; CI run: 45325569459 |
| 2026-11-24 | v0.2.6 | 隔离 mock 话题到 /mock/lite3 和 mock_* frames，避免污染真实 Lite3 topic/TF；修正双目同步与设备时钟方案；本地 freshness helper 6 项测试通过，ROS 2 CI 待结论 | mock: f9e37b65; launch: b7dbbfe; README: b5263ef3; latest CI: 45325875067 |
| 2026-11-24 | v0.2.7 | 更新 ROS 2 package 规划和 Lite3 bridge 接口契约，明确当前 scaffold 只读、mock 隔离、无真实 SDK/运动命令；根台账增加同步风险与 CI 状态 | workspace: 53997cdf; bridge contract: 44e69826 |
| 2026-11-24 | v0.2.8 | 将 CI 从易受 APT mirror 影响的裸机 setup-ros 安装切换到官方推荐的 Jammy/Noble 预制容器 + action-ros-ci | workflow: 2a218b84; run: 45326058355 |
| 2026-11-24 | v0.2.9 | ROS 2 bridge mock 包在 Humble/Jammy 与 Jazzy/Noble 双矩阵均构建成功，每边 6/6 pytest 通过；新增初步上装质量/功耗预算，A4 标记部分完成 | CI run 45326058355; power/mass budget: bcaa2116 |
| 2026-11-24 | v0.3.0 | 补充 JetPack 7.2.2 在 AGX Orin Developer Kit 上的安装路径细节；完善官方质量/功率证据和电源拓扑；更新双发行版 CI coverage 噪声处理 | ADR-0002: c60a263f; TaskPlan Part 1: 78546093; hardware checklist: 9e29d829; references: 3eed4ad5; workflow: b7dda179 |
| 2026-11-24 | v0.3.1 | 核验 Lite3 Legacy Foxy 与 current sdk_deploy 两条 ROS 路线，确认消息接口不同、DRDDS state 不是完整 Odometry，且跨 Foxy/Jazzy DDS 不应默认互通；新增 vendor integration assessment/ADR-0003 并更新桥接架构 | assessment: 0a0586c3; ADR-0003: 859390da; bridge: b5ca4f50; workspace: 78635dc7; architecture: c7c3ca5e; sensor fusion: eddefafb; references: f98192cd; CI run: 45326457632 |
| 2026-11-24 | v0.3.2 | 修正根 TaskPlan 和 Part 1 的本体状态估计前提：可信 legged odometry 需由当前 SDK/固件确认；若缺失须评估 LiDAR odometry/SLAM 或 stereo VIO，不能使用 IMU-only 积分冒充 odom | Part 1: 63bc92fb; root: 7f935a36 |
| 2026-11-24 | v0.3.3 | 建立 Lite3 Venture 供应商技术确认矩阵，将 A13 的 SDK/里程计/ROS distro/安全状态机/电源/载荷/网络时间同步问题整理成逐项证据清单 | vendor confirmation matrix: 68b7d857 |
| 2026-11-24 | v0.3.4 | 增加本体 odometry 缺失时的 LIO/VIO 备用设计：Point-LIO/MID-360 内置 IMU 首选 POC，FAST-LIO2 对照，Stereo VIO/FAST-LIVO2 后续评估；尚未实机验证 | odometry strategy: 55aa45a5; ADR-0004: e165237e |
| 2026-11-24 | v0.3.12 | 更新 MID-360、双目镜头/同步/USB 带宽、RTK 接入基线；核实 35/35 双发行版 CI 成功 | CI 45328982630；详见 stereo optics / RTK baseline |
| 2026-11-24 | v0.3.13 | 将两只 CIL530 候选镜头合计 334g 计入质量预算，候选总计调整为 1.664kg；其余未称重项和厂商载荷 Gate 保持未完成 | power_mass_budget.md commit 2ec892ebc6b44b789a47d792e7d555a7f7c302d9 |
| 2026-11-24 | v0.3.14 | 审计 Lite3 公共 SDK 状态字段/启动控制权/软件急停边界；补充 V-28–V-32 厂商问题；锁定 MID-360 Jazzy driver 与 SDK2 候选 commit，B4 build 保持待验证 | lite3 audit f9f8cec2; vendor matrix 90d7ab86; Livox baseline f5803ad7 |
| 2026-11-24 | v0.3.15 | 建立导航/避障验收指标和测量方法框架，A10 部分完成；新增 Livox Jazzy x86_64 build workflow，仍需等待结果并完成 Orin ARM64 验证 | acceptance metrics 30a81088; Livox CI run 45332455019 |
| 2026-11-24 | v0.3.16 | 将 Lite3 足端接触力纳入只读 telemetry envelope；contact_state 必须带 frame_id，新增 2 项测试；Humble/Jazzy 37 项 CI 双发行版通过 | contract 3341cc7b; tests fb463763; protocol 440ee4a4; CI 45332714322 |
| 2026-11-24 | v0.3.17 | 修复 Livox CI 缺少 ROS Jazzy 环境初始化的问题，改用 action-ros-ci 与 pin manifest；SDK2 锁定到 v1.3.1 并加入 GCC 13 cstdint 兼容参数，driver build 重测中 | workflow 21e3f186; manifest f4cf74ea; baseline ca297a9b; run 45333666724 |
| 2026-11-24 | v0.3.12 | 更新 MID-360、双目镜头/同步/USB 带宽、RTK 接入基线；核实 35/35 双发行版 CI 成功 | CI 45328982630；详见 stereo optics / RTK baseline |
| 2026-11-24 | v0.3.13 | 将两只 CIL530 候选镜头合计 334g 计入质量预算，候选总计调整为 1.664kg；其余未称重项和厂商载荷 Gate 保持未完成 | power_mass_budget.md commit 2ec892ebc6b44b789a47d792e7d555a7f7c302d9 |
| 2026-11-24 | v0.3.14 | 审计 Lite3 公共 SDK 状态字段/启动控制权/软件急停边界；补充 V-28–V-32 厂商问题；锁定 MID-360 Jazzy driver 与 SDK2 候选 commit，B4 build 保持待验证 | lite3 audit f9f8cec2; vendor matrix 90d7ab86; Livox baseline f5803ad7 |
| 2026-11-24 | v0.3.18 | 固定 Intel Jazzy Point-LIO 上游 gitlink SHA、三份 patch、Apache/BSD 许可证边界；新增受控源码准备脚本与 P0 CI；修正 Livox workflow 的容器内 manifest 路径 | baseline bf561d3d; prep script 4ae7a0c2; P0 workflow 74d61710; Livox workflow af0cb0bb |
| 2026-11-24 | v0.3.19 | Point-LIO P0 source-prep CI 通过：三个固定补丁成功应用，源码 tree clean；Livox workflow 改为 immutable HTTPS manifest 并重新运行，待 driver build 结果 | P0 run 45334159646; script 4ae7a0c2; Livox run 45334135171 |
| 2026-11-24 | v0.3.20 | 新增 Point-LIO P1 Jazzy x86_64 build script/workflow；setup-ros release tag 修正到 0.7.22；Livox manifest 修正 Driver2 v1.2.6 与 SDK2 v1.3.1 的 SHA 混用 | build script 6421e955; workflow 45ebd32e; manifest 8544a44b; Livox workflow 233d68ac |
