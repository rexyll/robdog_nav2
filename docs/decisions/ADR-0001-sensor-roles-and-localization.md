# ADR-0001：传感器职责分工与定位融合架构

- 状态：Proposed（待硬件基线确认）
- 日期：2026-11-24

## 背景

项目计划在四足机器狗上使用 LiDAR、RTK GNSS 和两台普通 RGB 2D 相机，基于 ROS 2 Nav2 实现导航避障。传感器数量有限，且其测量机理、可靠性和时空特性不同。必须明确谁负责局部几何、谁负责全局定位、谁提供视觉语义，并防止全局定位波动破坏局部控制。

## 决策

1. **LiDAR 是第一几何避障源。** 按实际设备发布 LaserScan 或 PointCloud2，优先用于 local costmap 和独立 Collision Monitor。
2. **两台普通 RGB 相机组成被动双目系统。** 提供深度点云和 RGB 语义，作为额外 obstacle source；不能仅凭“两台 2D 相机”就认定它们输出可靠深度。
3. **RTK GNSS 只负责全局位置约束和地理航点。** 不参与近距离碰撞检测；单天线位置不被假定为静止航向。
4. **Local EKF 与 Global EKF 分离。** local EKF 输出连续 `odom→base_link`；navsat_transform 将 GNSS 投影到 Cartesian frame；global EKF 输出 `map→odom`。
5. **LiDAR 和 Stereo 保持独立观测源进入 costmap。** 便于分别调整来源超时、高度、范围、清除行为和置信度。
6. **Nav2 负责底盘级路径规划和速度控制，不替代四足步态和落足规划。**
7. **Collision Monitor 作为软件附加保护。** 仍需要底层 watchdog 和硬件急停；不得宣称功能安全认证。

## 备选方案

### A. 仅用 LiDAR + RTK

优点：实现简单、几何稳定、系统算力和维护成本低。
缺点：缺少 RGB 语义；若是 2D LiDAR，存在扫描平面之外障碍物盲区；不擅长透明/负障碍问题。

### B. LiDAR + RTK + 双目 RGB（选择）

优点：同时保留 LiDAR 的几何测距、RTK 全局约束和视觉深度/语义；有机会补足部分高度/视场盲区。
缺点：双目标定、同步、算力和环境鲁棒性带来额外工程成本；玻璃、反光、无纹理、弱光仍会失败。

### C. 预先将所有点云融合成一个统一 cloud 再供 Nav2

优点：costmap 观测接口表面上简单。
缺点：难以追溯数据来源，统一噪声/置信度/时延假设不成立，清除行为容易产生错误自由空间判断。暂不采用，除非后续用数据证明其有明确收益。

## 后果

- 需要完整的 URDF/TF 与时间同步体系；
- 需要双目深度质量评估；
- 需要 GNSS 状态门控和定位降级状态机；
- 需要多 source costmap 和安全命令链；
- 四足地形功能需要额外模块，不能只依赖 Nav2 二维 costmap。

## 验证条件

本 ADR 在以下条件完成后可升级为 Accepted：
- 传感器型号/消息/时间戳已确认；
- local/global EKF 轨迹对比达到阶段 A 指标；
- LiDAR 与 Stereo 单独接入 local costmap 均有测试证据；
- 双目在目标场景完成有效深度范围/失效边界评测；
- Collision Monitor 与底层命令 watchdog 通过故障测试。
