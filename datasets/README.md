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
