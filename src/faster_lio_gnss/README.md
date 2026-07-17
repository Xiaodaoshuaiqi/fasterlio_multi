# faster_lio_gnss

ROS 1 package for causal online fusion of Faster-LIO, raw IMU, GNSS/RTK, and
geometrically verified loop closures.

The project overview, architecture figure, and benchmark plots are documented
in the [repository README](../../README.md).

## Runtime Nodes

| Executable | Responsibility |
|---|---|
| `online_pose3_fusion_node` | Online iSAM2 Pose3 factor graph and fused odometry |
| `nmea_replay_node` | Time-synchronized UrbanNav GGA replay as `NavSatFix` |
| `synthetic_rtk_replay.py` | Ground-truth plus deterministic centimeter noise |
| `online_ground_truth_replay.py` | Evaluation-only reference path publisher |
| `online_pose3_result_evaluator.py` | Automatic ATE/RPE report after bag completion |

Ground truth replay and evaluation nodes are used only by benchmark launch
files. The live fusion launch has no dependency on ground truth.

## Launch Files

### Live Sensors

Start sensor drivers and Faster-LIO first:

```bash
roslaunch faster_lio_gnss pose3_fusion_live.launch
```

Required inputs:

```text
/Odometry          nav_msgs/Odometry
/velodyne_points   sensor_msgs/PointCloud2
/imu/data          sensor_msgs/Imu
/gnss/fix          sensor_msgs/NavSatFix
```

Outputs:

```text
/odometry/pose3_online
/pose3_online/path/lio
/pose3_online/path/gnss
/pose3_online/path/fused
/pose3_online/loop_edges
/pose3_online/alignment_ready
TF: gnss_enu_pose3 -> odom
```

Example remapping:

```bash
roslaunch faster_lio_gnss pose3_fusion_live.launch \
  lio_topic:=/lio/odometry \
  cloud_topic:=/lidar/points \
  imu_topic:=/imu/data \
  gnss_topic:=/rtk/fix
```

### UrbanNav Benchmark

```bash
roslaunch faster_lio_gnss urbannav_final_test.launch \
  bag_file:=/path/to/test.bag \
  nmea_file:=/path/to/f9p.splitter.nmea \
  ground_truth_file:=/path/to/UrbanNav_TST_GT_raw.txt
```

The launch starts Faster-LIO, NMEA replay, Pose3 fusion, truth visualization,
RViz, rosbag playback, and automatic evaluation.

### Synthetic Centimeter RTK

```bash
roslaunch faster_lio_gnss urbannav_synthetic_rtk_test.launch \
  bag_file:=/path/to/test.bag \
  ground_truth_file:=/path/to/UrbanNav_TST_GT_raw.txt
```

Default synthetic noise:

```text
East/North sigma: 0.02 m
Height sigma:     0.04 m
Random seed:      20260716
```

Benchmark outputs are written below `~/.ros/faster_lio_gnss/`.

## State and Factors

Each keyframe contains:

```text
X(k)   Pose3
V(k)   world-frame velocity
BI(k)  IMU accelerometer and gyroscope bias
BG(k)  slowly varying 3D GNSS bias
```

The graph uses:

- Faster-LIO relative Pose3 factors;
- IMU preintegration and IMU bias random walk;
- LIO vertical-velocity consistency;
- robust horizontal GNSS antenna factors;
- robust GNSS height factors;
- GNSS bias random walk;
- GICP-verified robust loop factors;
- initial pose, velocity, and bias priors.

The first `warmup_seconds` are used for one fixed local-to-ENU alignment.
GNSS position factors are enabled only after alignment.

## Important Parameters

### Frames and Extrinsics

| Parameter | Meaning |
|---|---|
| `world_frame` | Global ENU fusion frame |
| `odom_frame` | Faster-LIO local frame |
| `antenna_x/y/z` | IMU reference point to GNSS antenna lever arm |
| `lidar_to_imu_z` | LiDAR-to-IMU vertical translation |

### GNSS

| Parameter | Meaning |
|---|---|
| `gnss_sigma_xy`, `gnss_sigma_z` | Horizontal and height observation noise |
| `gnss_bias_rw_xy/z` | GNSS bias random-walk density |
| `gnss_bias_prior_xy/z` | Initial GNSS bias prior |
| `maximum_horizontal_variance` | Covariance gate |
| `maximum_innovation` | Absolute LIO-predicted innovation gate |
| `maximum_increment_innovation` | Inter-fix increment gate |
| `consecutive_fixes` | Confirmation count before enabling GNSS |

### Loop Closure

| Parameter | Meaning |
|---|---|
| `loop_candidate_radius` | Spatial search radius |
| `loop_minimum_separation` | Minimum temporal separation |
| `loop_max_fitness` | Maximum accepted GICP fitness |
| `loop_max_correction` | Maximum accepted translation correction |
| `loop_translation_sigma` | Loop translation noise |
| `loop_rotation_sigma_degrees` | Loop rotation noise |

## Build

```bash
cd /path/to/fasterlio_multi
catkin_make -DCMAKE_BUILD_TYPE=Release --pkg faster_lio faster_lio_gnss
source devel/setup.bash
```

## Package Layout

```text
faster_lio_gnss/
├── config/
│   └── pose3_online_visualization.rviz
├── launch/
│   ├── pose3_fusion_live.launch
│   ├── urbannav_final_test.launch
│   └── urbannav_synthetic_rtk_test.launch
├── scripts/
│   ├── geodesy.py
│   ├── trajectory_metrics.py
│   ├── online_ground_truth_replay.py
│   ├── online_pose3_result_evaluator.py
│   └── synthetic_rtk_replay.py
└── src/
    ├── nmea_replay_node.cpp
    └── online_pose3_fusion_node.cpp
```
