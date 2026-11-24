# External evaluation datasets

Large datasets are intentionally not committed to this repository. Store them under
a local ignored directory and record source, license, checksum, acquisition date,
software version and conversion metadata in each evaluation report.

## UrbanLoco u l h k_4

- Session: `HK-Data20190117` (`ulhk_4` in the Intel Point-LIO reproduction)
- Preferred original source file: `datasets/ulhk_4/HK-Data20190117.bag` (ROS 1). The P2 audit tooling can inspect a standalone ROS 2 SQLite3 storage file such as `HK-Data20190117_0.db3`, but this file is not the expected UrbanLoco ROS 1 source archive. Topic/type matches do not prove that it was converted from the correct source sequence. It is diagnostic-only until its conversion/source provenance is documented; formal P2 replay/evaluation rejects it unless `URBANLOCO_DB3_PROVENANCE_CONFIRMED=1` is explicitly set after verification.
- Published size in UrbanLoco metadata: **6.11 GB**. Other Hong Kong sequences are listed at 24.0 GB, 27.9 GB, 41.6 GB and 62.3 GB; those are different sequences and must not be substituted for `ulhk_4`.
- The Dropbox and Baidu URLs below are shared-folder links for the Hong Kong datasets, not direct links to this single bag. Open the specific `HK-Data20190117` entry. If that entry is not visible, stop rather than downloading a different large bag; capture the folder's names/sizes so the entry can be identified before downloading.
- The official UrbanLoco GitHub README has a date-field inconsistency in this row (it labels the `HK-Data20190117` entry with `2019/03/16`); the official Hong Kong dataset page shows `2019/01/17`. Use the exact sequence ID `HK-Data20190117` as the identifying key.
- Expected converted ROS 2 bag: `datasets/ulhk_4/ulhk_bag/`, generated from the verified original ROS 1 `.bag` using `rosbags-convert`. A single `.db3` is an accepted rosbag2 SQLite storage input in Jazzy, but is not by itself proof of the expected UrbanLoco acquisition.
- Official dataset page: https://www.polyu.edu.hk/aae/ipn-lab/us/en/resources/urbanloco-dataset/
- License: CC BY-NC-SA 4.0. This is a non-commercial share-alike dataset license;
  confirm the project's intended use with the rights holder before commercial use
  or redistribution.
- Acquisition: manual download of the expected ROS 1 `.bag` from the official UrbanLoco Hong Kong dataset mirrors documented by the dataset and Intel's Point-LIO reproduction guide. Keep download record and SHA256.
- Expected source format/path follows Intel's Point-LIO guide: a ROS 1 `.bag` named `HK-Data20190117.bag`. If the mirror exposes an archive or differently named item, do not rename it blindly; verify its contents and report the actual name/size first.
- Preparation: `scripts/prepare_urbanloco_ulhk4.sh` (writes raw/DB3 SHA256, core-topic/INSPVAX audit and dataset manifest; standalone DB3 is marked unverified unless provenance is explicitly confirmed)
- Replay: scripts/run_urbanloco_ulhk4.sh
- Evaluation: scripts/evaluate_urbanloco_ulhk4.sh

Do not commit the raw .bag, converted .db3 files, rosbag2 metadata, trajectories,
or generated maps/PCD files.

## 替代公开数据集

UrbanLoco `ulhk_4` 的原始来源门禁仍保留；它不再是整个项目的唯一数据入口。GrandTour、Quadruped-SLAM 和 M3DGR 的优先级、适用边界、下载序列、许可和校验要求见 [外部公开数据集替代路线](../docs/datasets/alternative_dataset_strategy.md)。

## M3DGR Dynamic03

- Original ROS 1 bag: user download `~/Downloads/Dynamic03.bag`, size 3,434,689,371 bytes. Upstream SHA256 verified successfully on 2026-11-24: `A54B687A924EE4E9B30673F85D88AC73615A10FCE4A7AB40F6099C24283BCC78`.
- Matching GT is a separate file. Download only the Dynamic03 GT from [OneDrive](https://1drv.ms/t/c/2b4bfc0edf421186/EYYRQt8O_EsggCsWEAAAAAAB-G7sz7ob3JbPDr8K1eYEDg?e=8GwPON) or [Alipan](https://www.alipan.com/s/2GQnM2gePFr). The GT has not yet been downloaded/audited.
- Read-only ROS1 bag structure audit: `scripts/audit_m3dgr_ros1_bag.py`; local dependency: `requirements-dataset-tools.txt`; report JSON should be stored under ignored `datasets/m3dgr_dynamic03_audit.json`.
- Expected MID-360 inputs: `/livox/mid360/lidar` (`livox_ros_driver/CustomMsg`) and `/livox/mid360/imu` (`sensor_msgs/Imu`). Verify the actual bag connections before conversion.
- Do not reuse `m3dgr_mid360.yaml` blindly: the upstream M3DGR Point-LIO version uses `/camera/imu`; the upstream `mid360.yaml` uses MID-360's built-in `/livox/mid360/imu`. This project must adapt only after bag metadata and custom message definitions have been audited.
- The Dynamic03 bag byte checksum is verified; ROS structure, custom message decode, timestamp semantics, GT correctness, ROS 1→ROS 2 conversion and Point-LIO replay remain pending.

- Pinned upstream Livox ROS1 driver semantics for M3DGR commit `5fc892c84defef42feb2418f7d38897bef40f490`: CustomPoint `offset_time` and packet offset are nanoseconds; `CustomMsg.timebase` is first packet time and `header.stamp` derives from timebase / 1e9. This identifies the unit, **not** whether the device clock was PTP/PPS/NoSync.
- Dynamic03 GT is a separate download. Audit it first with `scripts/audit_tum_trajectory.py`; the tool validates TUM shape/finite values/time order/quaternion norm only and does not establish bag clock alignment or coordinate frame.
