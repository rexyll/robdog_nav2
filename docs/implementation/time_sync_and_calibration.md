# 多传感器时间同步与空间标定方案

- 状态：设计基线；传感器实物、时钟接口和同步误差尚未实测。
- 项目进度唯一台账：[根目录 TaskPlan.md](../../TaskPlan.md)
- 关联硬件决策：[ADR-0002](../decisions/ADR-0002-hardware-platform.md)

## 1. 为什么要区分不同层次的“同步”

本项目至少存在四个独立问题，不能统称为“PTP 已同步”：

1. **双目曝光同步：** 左右相机必须拍到同一时刻的场景，这是计算有效视差的前提。
2. **传感器时钟对齐：** LiDAR、相机、GNSS、Lite3 状态数据必须能映射到统一时间轴。
3. **机身运动补偿：** 传感器采集期间机器人发生平移、俯仰、横滚和转向时，点云/图像必须对应正确的机身姿态。
4. **ROS 消息时间戳：** header.stamp 应表达采集时间或经过测量/转换后的采集时刻，而不是不加区分地使用 callback 到达时间。

高质量的同步能降低空间融合误差，但无法替代外参标定和机械刚性安装。

## 2. 各设备的首选时间策略

| 设备 | 主要时间策略 | 需要实测的内容 |
|---|---|---|
| 两台 See3CAM_50CUG Color | 同一个外部硬件 trigger 源，触发左右相机曝光/采集 | 触发输入电平、边沿、延迟差、曝光开始偏差、驱动时间戳含义 |
| Livox MID-360 | 按官方支持的 PTPv2/GPS 时间功能评估，与 Jetson 主机网卡建立时钟关系 | 网卡是否支持硬件 timestamp/PHC、PTP offset、点云 header 时间语义 |
| Lite3 IMU/腿式 odom | 优先采用厂商原始采样时间戳；若只有 UDP 接收时间，测量延迟/抖动并标注为接收时间 | SDK 时间字段、主机/运动控制器时钟偏差、网络时延和数据频率 |
| ZED-F9P RTK | 保留 GNSS 时间/UBX 时间状态；若可用，将 TIMEPULSE/PPS 接到主机时间采集链 | PPS 电气接口、时间脉冲配置、UTC/GNSS 周时间、串口/USB 到达延迟 |
| Jetson AGX Orin | 由明确的主机时间服务统一 ROS 节点时间；若启用 PTP，先验证网卡 PHC 和 ptp4l/phc2sys 路径 | system clock 与 PHC 差异、PTP offset、网络负载下的抖动 |

## 3. 双目相机同步

### 3.1 硬件结构

    Common Trigger Controller
          /                 \
         /                   \
  Level-safe output A    Level-safe output B
        |                       |
  Left See3CAM GPIO       Right See3CAM GPIO
        |                       |
  Left UVC image          Right UVC image
         \                     /
          \                   /
           Stereo timestamp pairing
                      |
           Rectification / Disparity
                      |
               Depth / PointCloud2

- 两台相机使用相同 SKU、相同镜头规格、相同分辨率、相同曝光配置。
- 使用缓冲/扇出触发器，不要在未确认输入阻抗和驱动能力时简单并联 GPIO 线。
- 确认相机 trigger 电气电压、有效边沿、最小脉宽、曝光控制模式、strobe 输出和触发到曝光的延迟。
- 如果触发器没有统一时钟或时间戳，左右图像同步可以很好，但相机对 Jetson/LiDAR 的绝对时间仍需额外测量。
- 相机 UVC 驱动报告的 timestamp 可能是设备时刻，也可能是 host 到达时刻；必须通过 SDK/手册和高帧率实验确认。
- ROS message_filters ApproximateTime 只能匹配已有时间戳，不能修复曝光不同步。

### 3.2 相机同步验收

- 使用共同快速闪光/LED 脉冲或运动测试目标，对比左右图像中同一事件的帧编号。
- 记录触发延迟差、帧丢失、重复帧和帧率漂移。
- 以机器人最大计划角速度/线速度和最远目标距离推导可容忍时间偏差；不能只凭“图像看起来同时”验收。
- 在测试报告中给出 P50/P95/P99/max skew。
- 如果无法达到目标，则降低相机帧率/机器人速度、调整 trigger controller，或重新选支持可验证同步的相机。

## 4. LiDAR 与内置 IMU 时间域

Livox MID-360 产品规格列出 PTPv2/GPS 同步能力；但云深处 Lite3 FAST-LIVO2 开发说明对 Mid-360s 的 LiDAR 与内置 IMU 时钟差异给出了专门处理方案，并说明特定组合中 PTP/GPIO 不能直接解决内部 IMU 时钟漂移。

必须把以下结论分开：
- **MID-360 外部点云时间：** 按雷达输出协议和 PTP/GPS 配置核验。
- **雷达内部 IMU 时间：** 不能未经验证就认为与点云共用同一个已同步时钟。
- **Lite3 本体 IMU 时间：** 独立检查 SDK 时间戳，作为第一版本体状态估计的主要 IMU 来源。
- **两台 RGB 相机时间：** 由共同触发确保相互曝光同步，再与主机/雷达时钟建立 offset/漂移关系。

云深处示例特别讨论的是 Mid-360s + RealSense D435i + FAST-LIVO2，不应未经测试将其具体漂移速率/修正参数直接套用到本项目的 MID-360 原版和 See3CAM 双目组合。只借鉴“必须测量每个时间域、不可假定同步”的原则。

第一版 Nav2 障碍物融合不要求先运行 FAST-LIVO2。LiDAR、双目分别输出几何障碍观测，先完成时间戳映射、TF 和 source timeout。若以后采用紧耦合 LiDAR-Inertial-Visual SLAM，再单独实现/验证完整时间同步和运动补偿。

## 5. Lite3 状态消息时间戳

官方 SDK/ROS 2 示例给出腿式 odom、IMU、joint_states 和 cmd_vel，但不同 SDK 版本的时间戳语义和状态频率需要检查。

实施步骤：
1. 在不发送运动命令的前提下，记录 /leg_odom2、/imu/data、/joint_states 原始 header.stamp 和主机接收 monotonic time。
2. 识别时间戳是否随运动控制器采样时间增长、是否重置、是否有跳变。
3. 对连续录制的数据计算相邻消息间隔、时延、抖动和丢包。
4. 若消息没有源采样时间戳，明确其只能代表 ROS 接收时刻，不能用于精确 IMU/点云去畸变。
5. 不用虚构的静态 offset 掩盖未知时钟；先采集多段数据，再估计 offset/drift。

## 6. GNSS 时间与 RTK 位置分离

GNSS 的主要职责是位置约束。ZED-F9P 的 NavSatFix/UBX 数据和可配置时间脉冲可以用来建立主机时间关系，但必须确认板卡引脚、电气电平和配置。

- GNSS Fix/Float/Single 是位置解状态，不等同于时钟同步锁定状态。
- RTK correction age、fix type、position covariance 和 time-pulse validity 应分别诊断。
- 不能用收到 RTK Fix 就推断整个机器人所有 sensor clock 已同步。
- GNSS 时间脉冲线路必须有电平匹配/隔离和可靠接地，不可把未确认的 PPS 电压直接接到主机 GPIO。

## 7. 统一时间处理流程

    Device source timestamps
       |          |          |          |
      LiDAR     Camera      Lite3      GNSS
       |          |          |          |
    PTP/GPS   Shared GPIO   Vendor    PPS/GNSS time
       |      + host map   timestamp  + host mapping
       +----------+----------+----------+
                            |
                   Timestamp normalization
                            |
          Preserve source time + normalized ROS stamp
                            |
                TF lookup at measurement time
                            |
             motion compensation / synchronization
                            |
              Nav2 observation sources / EKF

具体实现规则：
- 原始时间戳和换算后时间戳都保留在诊断/rosbag 证据中。
- TF lookup 使用测量时间，不用最新 TF 代替历史 TF，除非已证明误差可接受。
- 需要扫描内去畸变时，应检查点云是否保留逐点时间信息；只有单个 header 时间戳不能自动完成逐点 deskew。
- 消息 age/freshness 监测用主机 monotonic clock；ROS header 使用统一后的 ROS time。
- 仿真启用 use_sim_time 时，时间 freshness 仍要区分仿真时钟和主机接收时间。

## 8. 标定项目

### 相机内部
- 左、右相机分别标定 fx/fy/cx/cy 和畸变；
- 双目标定得到 R/T、baseline、rectification 和 projection；
- 改变分辨率、ROI、镜头或焦点/镜头锁紧状态后，必须重新验证标定。

### 设备外部
- left/right camera optical frame 到 base_link；
- MID-360 到 base_link；
- GNSS 天线相位中心到 base_link 的 lever arm；
- Lite3 IMU 到 base_link 的安装旋转；
- 有条件时进行 LiDAR-camera 外参标定。

### 动态验证
- 静止、直行、横移（若支持）、原地转弯、快速转向和步态切换；
- 检查点云/图像中的固定地面/墙面在统一坐标系中是否发生系统性重影；
- 重新紧固上装后检查外参漂移。

## 9. 必须保存的证据

- 原始相机帧、触发脉冲或 trigger counters；
- MID-360 原始点云/IMU、PTP 状态；
- Lite3 odom/IMU/joint 原始消息；
- RTK NavSatFix/UBX 状态/PPS 记录；
- ROS /tf、/tf_static、rosbag2；
- host PTP/PHC 状态和系统时间状态；
- 统计 skew/offset/drift、丢帧和延迟分布；
- 标定图像、标定板尺寸、参数和结果文件。

没有上述测量结果前，不把时间同步标记为完成。
