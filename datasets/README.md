# External evaluation datasets

Large datasets are intentionally not committed to this repository. Store them under
a local ignored directory and record source, license, checksum, acquisition date,
software version and conversion metadata in each evaluation report.

## UrbanLoco u l h k_4

- Session: HK-Data20190117
- Expected raw file: datasets/ulhk_4/HK-Data20190117.bag
- Expected converted ROS 2 bag: datasets/ulhk_4/ulhk_bag/
- Official dataset page: https://www.polyu.edu.hk/aae/ipn-lab/us/en/resources/urbanloco-dataset/
- License: CC BY-NC-SA 4.0. This is a non-commercial share-alike dataset license;
  confirm the project's intended use with the rights holder before commercial use
  or redistribution.
- Acquisition: manual download from the official UrbanLoco Hong Kong dataset
  mirrors documented by the dataset and Intel's Point-LIO reproduction guide.
- Preparation: scripts/prepare_urbanloco_ulhk4.sh (writes raw/DB3 SHA256 and dataset_manifest.txt)
- Replay: scripts/run_urbanloco_ulhk4.sh
- Evaluation: scripts/evaluate_urbanloco_ulhk4.sh

Do not commit the raw .bag, converted .db3 files, rosbag2 metadata, trajectories,
or generated maps/PCD files.
