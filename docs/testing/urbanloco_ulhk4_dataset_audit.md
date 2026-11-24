# Standalone ROS 2 SQLite3 storage-file audit — UrbanLoco identity unverified

- Audit date: 2026-11-24
- Sequence candidate only: `HK-Data20190117` / Intel Point-LIO `ulhk_4` (not confirmed)
- User-provided input: `~/Downloads/HK-Data20190117_0.db3`
- Input kind: standalone ROS 2 rosbag2 SQLite3 storage file. ROS 2 Jazzy can read/play this format directly, but the expected UrbanLoco download in Intel's Point-LIO instructions is the original ROS 1 `HK-Data20190117.bag`; the supplied DB3 is not that original file.
- File size: 5,830,840,320 bytes (reported by rosbag2 as 5.4 GiB)
- rosbag2 duration: 321.550093281 s
- Start / end: 1547714239.744714975 / 1547714561.294808256 (bag clock)
- Total messages: 41,817
- Local source path is not embedded as a portable project path. The file must remain outside Git; no DB3/raw bag is committed.
- **Acquisition gate: NOT SATISFIED.** The user-provided DB3 has matching topic/type signatures and valid INSPVAX payloads, but there is no confirmed source URL/download record or verified conversion chain from the official ROS 1 `HK-Data20190117.bag`. These checks cannot establish exact sequence identity. This report is a storage-file diagnostic only, not proof that the required source dataset has been acquired. Record DB3 SHA256 and source acquisition/conversion chain separately.

## Required topic/type and temporal summary

The reported rates are computed from each topic's own first/last message timestamps and count, not from ROS playback wall time.

| Topic | Type | Messages | First-to-last span | Average rate | Duplicate / backward timestamps |
|---|---|---:|---:|---:|---:|
| `/velodyne_points_0` | `sensor_msgs/msg/PointCloud2` | 3,178 | 317.979333120 s | 9.991215 Hz | 0 / 0 |
| `/imu/data` | `sensor_msgs/msg/Imu` | 31,808 | 318.072230100 s | 99.999299 Hz | 0 / 0 |
| `/novatel_data/inspvax` | `novatel_oem7_msgs/msg/INSPVAX` | 318 | 317.115849447 s | 0.999635 Hz | 0 / 0 |

The complete bag start/end span is 321.550093281 s. The three core streams each cover approximately 317–318 seconds; the shorter topic-specific coverage is expected to be considered explicitly during trajectory alignment and APE/RPE evaluation.

## INSPVAX CDR payload validation

- All 318 payloads were decoded by the project audit with the fixed UrbanLoco HK-Data20190117 little-endian float64 field offsets:
  - latitude: byte offset 60
  - longitude: byte offset 68
  - height: byte offset 76
- Observed latitude range: 22.330298794556676–22.331954627824523 degrees.
- Observed longitude range: 114.17951676205799–114.18085578754544 degrees.
- Observed height range: 15.863546616397798–21.74851658102125 m.
- All values passed finite/range validation.
- The offsets are independently pinned to NovAtel's `INSPVAX.msg` and Intel's fixed UrbanLoco GT extraction script. This does not validate the statistical accuracy, covariance, solution status, or temporal alignment of the NovAtel solution.

## Current verdict and limitations

**Passed (storage-file diagnostics only):** SQLite rosbag2 storage structure, required topic names/types, non-empty message streams, selected-topic timestamp ordering (zero duplicate/backward timestamps), and all INSPVAX latitude/longitude/height payload parsing. This does not satisfy the original UrbanLoco ROS 1 source-file acquisition gate.

**Not yet established:** identity/provenance of the supplied DB3; existence and SHA256 of the expected original ROS 1 bag; a documented ROS1→ROS2 conversion chain; PointCloud2/IMU payload content/field layout; formal Point-LIO replay; estimated trajectory completeness; timestamp overlap/interpolation; evo APE/RPE; Jetson Orin and robot hardware.

The initial interactive `ros2 bag play` provided by the user stopped approximately 56 seconds after start, less than the 321.55-second bag duration. It is not evidence of a complete playback. The P2 runner filters playback to the required LiDAR and IMU topics. It now rejects standalone DB3 for formal P2 replay/evaluation unless the user explicitly confirms its conversion provenance after verification. Do not set that override merely because ros2 bag info/play succeeds.

UrbanLoco is a historical Velodyne dataset. Even a successful P2 replay and low APE/RPE demonstrate only an offline algorithm regression on this dataset. It does not validate MID-360 per-point timing, Livox/IMU PTP synchronization, Jetson AGX Orin performance, Lite3 gait motion, or safe autonomous navigation.
