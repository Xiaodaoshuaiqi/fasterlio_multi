# Faster-LIO Front-End

This directory contains the Faster-LIO LiDAR-inertial odometry front-end used
by the GNSS fusion project.

Upstream project:
[gaoxiang12/faster-lio](https://github.com/gaoxiang12/faster-lio)

Faster-LIO is a lightweight tightly coupled LiDAR-inertial odometry system
based on Fast-LIO2 and parallel sparse incremental voxels. The original license
is preserved in [`LICENSE`](LICENSE).

## Project-Specific Changes

The upstream algorithm is intentionally kept intact. Only the integration
surface required by the online GNSS back-end was changed:

| File | Change |
|---|---|
| `app/run_mapping_online.cc` | Uses `ros::WallRate` so shutdown remains responsive after simulated `/clock` stops |
| `src/laser_mapping.cc` | Publishes odometry after covariance is populated and reports point-time scale |
| `config/velodyne_ulhk.yaml` | UrbanNav topics, calibrated LiDAR-IMU transform, frames, and safe PCD settings |

The public interface consumed by `faster_lio_gnss` is:

```text
Input:
  /velodyne_points   sensor_msgs/PointCloud2
  /imu/data          sensor_msgs/Imu

Output:
  /Odometry          nav_msgs/Odometry
  registered clouds and TF in the odom frame
```

## Build

```bash
cd /path/to/fasterlio_multi
catkin_make -DCMAKE_BUILD_TYPE=Release --pkg faster_lio
source devel/setup.bash
```

Dependencies:

- ROS Noetic
- Eigen3
- PCL
- yaml-cpp
- glog

## UrbanNav

Run the front-end only:

```bash
roslaunch faster_lio mapping_velodyne_ulhk.launch
```

Then play a bag containing:

```text
/velodyne_points
/imu/data
```

For the complete online localization pipeline, use:

```bash
roslaunch faster_lio_gnss urbannav_final_test.launch \
  bag_file:=/path/to/test.bag \
  nmea_file:=/path/to/f9p.splitter.nmea \
  ground_truth_file:=/path/to/UrbanNav_TST_GT_raw.txt
```

## Upstream Citation

```bibtex
@article{bai2022fasterlio,
  title   = {Faster-LIO: Lightweight Tightly Coupled Lidar-Inertial
             Odometry Using Parallel Sparse Incremental Voxels},
  author  = {Bai, Chunge and Xiao, Tao and Chen, Yajie and Wang, Haoqian
             and Zhang, Fang and Gao, Xiang},
  journal = {IEEE Robotics and Automation Letters},
  volume  = {7},
  number  = {2},
  pages   = {4861--4868},
  year    = {2022},
  doi     = {10.1109/LRA.2022.3152830}
}
```
