# UrbanLoco HK-Data20190117 standalone DB3 input audit

- Audit date: 2026-11-24
- Sequence candidate: `HK-Data20190117` / Intel Point-LIO `ulhk_4`
- User-provided input: `~/Downloads/HK-Data20190117_0.db3`
- Input kind: ROS 2 rosbag2 SQLite3 standalone database; this is not the original ROS 1 `.bag`
- File size: 5,830,840,320 bytes (reported by rosbag2 as 5.4 GiB)
- rosbag2 duration: 321.550093281 s
- Start / end: 1547714239.744714975 / 1547714561.294808256 (bag clock)
- Total messages: 41,817
- Local source path is not embedded as a portable project path. The file must remain outside Git; no DB3/raw bag is committed.
- Provenance: the topic/type and CDR signatures below are consistent with the project's fixed HK-Data20190117 contract, but the dataset source and exact sequence identity are not cryptographically proven by these fields. Record the DB3 SHA256 and source acquisition record separately.

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

**Passed:** SQLite rosbag structure, required topic names, required message types, non-empty message streams, selected-topic timestamp ordering (zero duplicate/backward timestamps in the audit), and all INSPVAX latitude/longitude/height payload parsing.

**Not yet tested:** PointCloud2 and IMU payload deserialization/content/field layout; Point-LIO full replay; estimated trajectory completeness; INSPVAX/estimated trajectory time-overlap and interpolation policy; evo APE/RPE; source checksum and acquisition provenance; Jetson Orin and actual robot hardware.

The initial interactive `ros2 bag play` provided by the user stopped approximately 56 seconds after start, less than the 321.55-second bag duration. It is not evidence of a complete playback. The P2 runner now filters playback to the required LiDAR and IMU topics and records the actual replay result.

UrbanLoco is a historical Velodyne dataset. Even a successful P2 replay and low APE/RPE demonstrate only an offline algorithm regression on this dataset. It does not validate MID-360 per-point timing, Livox/IMU PTP synchronization, Jetson AGX Orin performance, Lite3 gait motion, or safe autonomous navigation.
