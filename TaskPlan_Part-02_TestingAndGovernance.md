# RobDog Nav2 总任务计划 — 测试、风险与参考资料

- 对应根文件：[TaskPlan.md](TaskPlan.md)
- 本分册为稳定设计细节，不维护任务完成状态；所有进度以根目录 TaskPlan.md 为准。

## 9. 量化验收和测试矩阵

数字指标在 A9 根据实际运行速度和风险分析确定。至少采集以下指标：

| 类别 | 指标 |
|---|---|
| 定位 | RTK Fixed/Float 误差，local odom 漂移，yaw/roll/pitch 误差，map→odom 连续性 |
| 感知 | 目标类别/尺寸/距离召回率，漏检率，误检率，深度 RMSE，首次检测距离 |
| 时延 | 相机同步误差，LiDAR 时间误差，perception latency，costmap update latency，cmd latency |
| 避障 | 最小间隙，停车时间/距离，动态行人响应，重规划成功率 |
| 鲁棒性 | 断流、过期数据、TF 丢失、GNSS 降级、遮挡、振动 |
| 资源 | CPU/GPU/NPU、RAM、带宽、温度、功率和续航 |
| 地形 | 坡度/台阶/沟坎检测率，可通行性分类，足端/机身稳定性 |

测试级别：
1. 静态数据与接口检查；
2. 离线 rosbag 感知评测；
3. 定位回放与真值对比；
4. 仿真/HIL；
5. 断电/断网/传感器超时故障注入；
6. 急停和 watchdog 验证；
7. 封闭场地站立、低速、逐级速度测试；
8. 受控户外 RTK 测试；
9. 复杂地形测试（仅在底层落足能力确认后）。

实机自主运动前必须有物理急停/遥控停止、现场监护、隔离测试区域和底层命令超时停止。任何无法解释的运动命令或漏停立即停止测试。

---

## 10. 主要风险和处理策略

| 风险 | 影响 | 处理策略 |
|---|---|---|
| Lite3 Venture 外设电流不够 | AGX/雷达掉电或重启 | 厂商确认电流；单独 DC/DC、电源分配和保护 |
| 总载荷超 4.5 kg | 续航、姿态和结构风险 | 内部目标 ≤3.6 kg，逐项称重，必要时更换计算载板 |
| 厂商 Humble 栈与 Jazzy 目标冲突 | 版本/依赖兼容风险 | 先 Humble 复现，再 adapter 隔离和 Jazzy 迁移 |
| 双目左右未同步 | 深度误差大 | 共触发、曝光控制、量测同步误差 |
| 相机镜头视场过窄/畸变大 | 视野缺失/深度差 | 镜头选型、视场和标定误差联合验证 |
| RTK 多路径/遮挡 | 全局定位跳变 | quality gate，失效降级，不参与近场避障 |
| 单天线 RTK 无静态航向 | 低速航向不可信 | 用 IMU/leg odom；必要时双天线升级 |
| 机身步态振动/俯仰横滚 | 点云/双目/TF 失配 | 硬件刚性安装、同步、IMU motion compensation |
| 2D Nav2 不理解足端落点 | 地形碰撞/跌落 | 第一阶段只做平地；F 阶段增加 traversability/locomotion |
| Collision Monitor 不是安全认证系统 | 误以为已安全 | 独立硬件急停、底层 watchdog 和风险分析 |

---

## 11. 采购前供应商确认清单

向 DEEP Robotics 确认：
- Lite3 Venture 是否仍在售，当前 SKU、固件、SDK 和保修；
- 官方 AGX 安装套件是否适用于当前批次、可提供 CAD/安装图；
- MID-360 24V XT30 和 Ethernet 的官方接法；
- 外部 24V/12V/5V 每路电流和总功率；
- 4.5kg payload 对上装重心、动态步态和长期运行的限制；
- Lite3 状态数据频率/时间戳和 SDK command timeout。

向 e-con Systems 确认：
- 两台 See3CAM_50CUG Color 采用相同 SKU、固件和相同镜头；
- GPIO 触发电气规格、触发延迟和左右曝光同步能力；
- Linux/Jetson UVC 兼容性及双相机同时 5MP@目标帧率的 USB 带宽；
- 2/3-inch C-mount 对应 70°–80° HFOV 镜头推荐和畸变数据。

向 ArduSimple 确认：
- ZED-F9P 与 ANN-MB 天线适用于目标国家/地区的卫星频段；
- NTRIP 改正服务覆盖/费用，或自建基站需求；
- USB/UART 配置、输出频率和 RTK status diagnostics。

---


## 13. 重要参考资料

### 机器狗与官方接口

- DEEP Robotics Lite3 产品页：https://www.deeprobotics.us/products/lite-3/
- DEEP Robotics Lite3 感知算法开发示例（Lite3 Venture + AGX Orin + MID-360 + Ubuntu 22.04 + ROS 2 Humble）：https://developer.deeprobotics.cn/docs/lite3/examples/sdk-dev/perception-algorithm-example/
- Lite3 ROS 2 Transfer：https://github.com/DeepRoboticsLab/Lite3_ROS
- Lite3 SDK Deploy：https://github.com/DeepRoboticsLab/sdk_deploy

### 传感器与计算平台

- Livox MID-360 规格：https://www.livoxtech.com/mid-360/specs
- Livox ROS Driver 2：https://github.com/Livox-SDK/livox_ros_driver2
- See3CAM_50CUG IMX264 相机：https://www.e-consystems.com/sony-imx264-camera.asp
- ArduSimple simpleRTK2B Budget Starter Kit：https://www.ardusimple.com/product/simplertk2b-basic-starter-kit-ip65/
- NVIDIA Jetson AGX Orin Developer Kit：https://developer.nvidia.com/embedded/jetson-agx-orin-developer-kit

### ROS 2 / Nav2

- ROS 2 REP-2000：https://www.ros.org/reps/rep-2000.html
- ROS 2 Jazzy 发布和支持期：https://www.openrobotics.org/blog/2024/5/ros-jazzy-jalisco-released
- Nav2 文档：https://docs.nav2.org/
- Nav2 GPS Navigation：https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html
- robot_localization：https://github.com/cra-ros-pkg/robot_localization

---

