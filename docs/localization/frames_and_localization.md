# 坐标系、TF 与定位设计

## 1. 推荐 TF 树

```text
earth (optional)
  └── map
       └── odom
            └── base_link
                 ├── base_footprint (if used, follow one chosen convention)
                 ├── imu_link
                 ├── lidar_link
                 ├── left_camera_link
                 │    └── left_camera_optical_frame
                 ├── right_camera_link
                 │    └── right_camera_optical_frame
                 └── gnss_antenna_link
```

- `map → odom`：由 global localization（global EKF/SLAM/localization）唯一发布。
- `odom → base_link`：由 local state estimator 唯一发布。
- `base_link → sensor frame`：由 robot_state_publisher/URDF 或静态 TF 唯一发布。
- `base_footprint` 的使用要和 Nav2 参数一致。对于四足机身，需定义它是水平地面投影框架还是机身刚体框架，不能把两种语义混用。
- 若启用 REP 105 earth frame，必须检查 navsat_transform 的 Cartesian/earth transform 与 map/odom 的层级，不得产生第二条重复 TF 链。

## 2. 坐标约定

遵循 ROS 常用约定：
- 右手系，长度米，角度弧度；
- 机体通常 x 前、y 左、z 上；
- map/odom 尽量使用 ENU（x East, y North, z Up）或在接口文档中清楚声明其他约定；
- 相机 optical frame 按 ROS 规范使用 z forward、x right、y down；
- IMU 驱动需明确原始轴向和转换过程；
- GNSS 坐标 WGS-84 经纬度/椭球高到 UTM 或 Local ENU 的变换必须统一。

## 3. 两层 EKF 的职责

### Local EKF（world_frame = odom）

典型输入：
- 腿式状态估计器输出的 pose/twist；
- IMU 角速度、线加速度或经过验证的 orientation；
- 可选的足端/滑移状态转换成可信 odometry 修正。

输出：
- `/odometry/filtered/local`；
- `odom → base_link`。

关键：若厂商腿式 odometry 已经融合 IMU，不能再将高度相关的 IMU 信息当作独立、统计独立的第二个测量简单融合。需要根据其估计器设计、协方差和数据相关性选择状态变量。

### navsat_transform_node

输入：
- local filtered odometry；
- 有地理参考意义的 IMU yaw，或符合条件的姿态/航向来源；
- `NavSatFix`。

输出：
- `/odometry/gps`，将地理定位转入当前 world frame 的笛卡尔坐标。

需要锁定：
- 使用 UTM 还是 local ENU；
- datum 是启动时首个 Fix 还是显式固定基准；
- magnetic declination、yaw offset、heading frame；
- GNSS 天线到 base_link 的杆臂；
- UTC/GNSS/ROS 时间的转换；
- GNSS 质量过滤与跳变门限。

### Global EKF（world_frame = map）

输入：
- local filtered odometry（连续的相对运动）；
- 经质量门控的 `/odometry/gps`（绝对位置约束）；
- 如有必要，经过正确 frame/covariance 配置的姿态观测。

输出：
- `/odometry/filtered/global`；
- `map → odom`。

注意：如果系统还使用 SLAM/地图匹配，则只能由一个明确的定位仲裁方案发布 map→odom。不能让 AMCL、global EKF、SLAM 同时无协调地发布同一 TF。

## 4. GNSS 质量状态机

建议建立可观测的 GNSS 状态：
- `RTK_FIXED`
- `RTK_FLOAT`
- `GNSS_DEGRADED`
- `GNSS_LOST`
- `RECOVERING`

策略示意：
- Fixed 且时间新鲜、协方差可信：按已标定协方差融合。
- Float/单点：依据阶段 A 确定的阈值降权或停止融合；不能仍沿用 Fixed 的误差协方差。
- 超时/状态无效/innovation 过大：丢弃位置更新，继续依赖 local odom 短期导航并增加不确定度。
- 超过允许的无 GNSS 漂移时间或定位协方差上限：限速或停车。
- 恢复时先验证位置一致性、状态持续时间和航向，再平滑恢复融合，避免位置突跳。

具体门限必须由接收机状态、测量数据和测试确定。

## 5. RTK 航向注意事项

- 单天线 GNSS 主要提供位置。机器人静止时无法仅由一个天线的位置点得到机身朝向。
- 运动方向只有在速度足够、侧滑较小且方向与机身轴线一致时才可能近似 heading；四足侧移/转身/原地转向时尤其不成立。
- 推荐来源：已校准的 IMU 绝对航向、磁航向（评估电机/电流磁干扰）、双天线 GNSS heading，或经过验证的其他 heading estimator。
- `navsat_transform_node` 的 yaw offset、磁偏角和 IMU 轴向必须实测确认。

## 6. Nav2 坐标选择

- Global Costmap：通常 `map`。
- Local Costmap：通常 `odom`，rolling window。
- Robot base frame：统一采用 `base_link` 或明确约定的 `base_footprint`。
- Sensor data：原始消息 `header.frame_id` 对应实际传感器坐标，不能为了省 TF 而伪造 frame_id。
- transform_tolerance 要按实际时延和 TF 发布率设置，不可只为消除报错而无限增大。

## 7. 定位验收

至少覆盖：
- 静止多路径环境下 RTK fixed/float；
- 直线行走、横移（如支持）、原地转向、曲线和加减速；
- 地面摩擦变化、足滑和步态切换；
- RTK 遮挡、断改正、恢复；
- GNSS 与 IMU/odom 时间错位；
- 长距离累计漂移和重新启动 datum 一致性；
- map→odom 与 odom→base_link 变换连续性。

每项保留 rosbag、配置版本、真值来源、统计指标和失败样例。
