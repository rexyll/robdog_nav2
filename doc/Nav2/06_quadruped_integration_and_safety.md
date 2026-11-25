# 06 — 四足机器人集成与安全架构

## 1. Nav2 速度不是四足关节命令

Nav2 主要输出平面速度意图（Twist/TwistStamped）。四足控制器负责姿态稳定、步态相位、支撑切换、足端轨迹、接触力和关节闭环。二者之间必须有平台专用 adapter，不允许直接把 Nav2 输出映射为关节目标。

## 2. 推荐命令链

    BT / Controller / Behavior
             │ cmd_vel_nav
             ▼
       Velocity Smoother
             │ cmd_vel_smoothed
             ▼
       Collision Monitor
             │ cmd_vel_safe
             ▼
       robodog_cmd_adapter
         - message type / timestamp validation
         - mode & gait state gate
         - velocity / acceleration / jerk limits
         - unsupported-axis rejection
         - freshness / sequence checks
         - stop / hold / stand policy
         - vendor transport watchdog
             │
             ▼
       Lite3 locomotion controller → actuators

所有恢复行为、遥控辅助或应用命令都必须进入同一安全命令汇聚点，不能旁路覆盖。

## 3. 运动能力冻结

向厂商确认并通过低风险测试：x/y/yaw 能力；横移是否真实支持；原地转向/倒退/斜向；速度/加速度/角速度；命令坐标系、单位、时间戳和保持时长；零速度/站立/阻尼/急停的语义；步态切换延迟；控制权限；断链 watchdog。

Nav2 holonomic、Controller 采样空间、Velocity Smoother limits、Collision Monitor 多方向区域必须匹配真实能力。未知维度禁用，不得猜测。

## 4. 状态反馈

优先使用经验证的 base odometry；核对 pose/twist 定义、frame、时间戳、协方差、频率、漂移和足端滑移。IMU + joint states 不自动等于完整 base odometry。若厂商无可信 odom，按 TaskPlan 评估 Point-LIO + MID-360 IMU。可选反馈还包括 contact、姿态、电池、温度、controller mode 和 estimator status。

## 5. Footprint 与地形

Footprint 覆盖机身、传感器支架和突出附件，并评估摆动腿/动态外形。机身姿态变化要求点云使用采集时刻 TF。二维 Costmap 不直接解决落足点、坡度、台阶、沟槽、悬崖、松软地面和足端稳定性；复杂地形需 elevation/traversability、foothold planner 与 gait controller 协作。首轮限制在已验证平整地面/缓坡。

## 6. Collision Monitor 与安全边界

Collision Monitor 可基于 LaserScan、PointCloud2、Range，在 Costmap/Controller 之外执行 stop、slowdown、limit、approach 区域检查。它是 CPU 软件级保护，不是 safety-rated scanner/controller，也不是 SIL/PL 认证急停。

停车距离估算：
    d_stop = v * t_reaction + v^2 / (2 * a_brake) + d_margin

t_reaction 包括传感器周期、处理/传输、Collision Monitor 周期、命令链延迟和底层响应；a_brake 必须由机器狗在不同地面/负载/步态下测量。默认参数不能替代实测。

必须保留独立硬件急停/驱动禁能/遥控接管；软件监控失效、节点崩溃或传感器断流时，底层 watchdog 进入预定义安全状态。

## 7. 故障降级

| 条件 | 上层策略 | 底层策略 |
|---|---|---|
| LiDAR stale | 不允许旧数据 clearing；限制/停止 | watchdog |
| Stereo stale/低置信 | 移除观测并告警 | 限速 |
| Odom stale/跳变 | cancel navigation | stop/stand |
| TF missing | 拒绝错误 frame | stop |
| GNSS 非 Fixed | 降级/暂停地理航点 | local safety |
| Costmap timeout | 不继续用旧地图 | stop |
| Collision Monitor source timeout | 视为安全观测缺失 | stop/hold |
| Nav2 crash | supervisor 检测失败 | 独立 watchdog |
| Adapter 断链 | 命令不可达 | 底层自动超时停止 |
| 人工急停 | 标记任务 aborted/disabled | 硬件优先切断 |

## 8. 实机 Gate

未取得厂商只读 bring-up SOP、SDK 控制权、物理急停/遥控接管、速度/制动参数和场地风险评估前，不启动可能抢占控制权的运动 SDK，不发送真实速度/关节命令。文档/CI/仿真不能替代封闭场地低速验证和安全审查。

参考：https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring-collision-monitor-node/
