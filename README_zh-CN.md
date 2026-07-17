<div align="center">

# Faster-LIO GNSS Fusion

### 面向城市峡谷的在线激光-惯性-GNSS鲁棒定位系统

Faster-LIO 前端 · GTSAM iSAM2 后端 · 鲁棒 GNSS 因子 · GICP 回环检测

[English](README.md) | [简体中文](README_zh-CN.md)

</div>

---

本仓库实现了一套完整的 ROS 1 多传感器定位与建图系统。系统以
Faster-LIO 作为激光惯性前端，在其输出之上构建在线、因果的 Pose3 因子图，
用于融合激光雷达、独立 IMU、GNSS/RTK 和几何回环约束。

系统主要面向高楼、树荫、隧道出入口等复杂环境。在这些场景中，GNSS 可能
出现多路径误差、慢变偏差、观测跳变、短时失锁或更新频率不足等问题。

项目已在 `UrbanNav-HK-Medium-Urban-1` 数据集上完成验证。SPAN-CPT + IE
真值只用于显示和误差评估，**不会作为因子加入在线融合图**。

<p align="center">
  <img src="docs/assets/system_architecture.png" width="96%" alt="系统架构图">
</p>

## 核心能力

- 高频激光惯性里程计与三维点云建图。
- 基于 iSAM2 的在线增量式 Pose3 因子图优化。
- 同时估计三维位姿、速度、IMU 零偏和 GNSS 慢变偏差。
- 融合水平 GNSS、高度、IMU 预积分、LIO 相对位姿和 GICP 回环因子。
- GNSS 协方差、速度、创新量、增量方向和连续多点门控。
- 前 60 秒只进行固定局部坐标系到 ENU 坐标系的初始化对齐。
- 支持 UrbanNav 真实 F9P 基准测试与厘米级 RTK 上限实验。
- 支持 RViz 三维可视化，并在 rosbag 播放结束后自动输出 ATE/RPE。

## 因子图结构

第 `k` 个关键帧包含以下优化状态：

<p align="center">
  <img src="docs/assets/factor_graph_state.png" width="72%" alt="因子图状态定义">
</p>

其中：

- `T_wb,k`：机器人在全局坐标系下的三维位姿；
- `v_w,k`：机器人在全局坐标系下的速度；
- `b_imu,k`：IMU 加速度计和陀螺仪零偏；
- `b_gnss,k`：三维 GNSS 慢变偏差。

| 因子 | 作用 |
|---|---|
| LIO 相对位姿因子 | 保留 Faster-LIO 提供的高频局部运动和几何约束 |
| IMU 预积分因子 | 约束三维运动、速度、重力方向和 IMU 零偏 |
| 水平 GNSS 因子 | 修正长期水平漂移，建立全局位置约束 |
| GNSS 高度因子 | 独立约束垂直方向的长期漂移 |
| GNSS 偏差随机游走 | 描述城市峡谷中的慢变定位偏差 |
| GICP 回环因子 | 在重访区域减少累计漂移 |
| 速度一致性因子 | 使用 LIO 垂直速度稳定 IMU 预积分状态 |

GNSS 观测只有通过独立质量检查后才会进入因子图。GNSS 因子使用 Huber
鲁棒核，回环因子使用 Cauchy 鲁棒核，以降低异常观测对整条轨迹的影响。

## UrbanNav 实验结果

误差评估只使用前 60 秒估计一次固定 SE(2) 对齐和一个垂直零点偏移。完成
初始对齐后，不再利用真值修正轨迹。

| 系统 | 水平 RMSE | 垂直 RMSE | 三维 RMSE | 100 m RPE |
|---|---:|---:|---:|---:|
| Faster-LIO | 6.108 m | 20.395 m | 21.290 m | 2.421 m |
| F9P NMEA | 5.119 m | 5.869 m | 7.788 m | 2.154 m |
| **Pose3 融合** | **0.550 m** | **1.252 m** | **1.368 m** | **0.292 m** |

<p align="center">
  <img src="docs/assets/urbannav_real_gnss_trajectory.png" width="96%" alt="UrbanNav真实GNSS轨迹对比">
</p>

该数据集位于香港典型城市峡谷，F9P 原始定位为米级误差。融合后的水平
RMSE 相比 F9P 降低约 `89.2%`，同时能够在两次 GNSS 更新之间持续输出
高频六自由度状态。

### 厘米级 RTK 上限实验

为了分析系统在高质量 RTK 输入下的性能上限，实验从参考轨迹生成确定性
模拟 RTK：

- 东向和北向分别加入 `2 cm` 标准差的高斯噪声；
- 高度加入 `4 cm` 标准差的高斯噪声；
- 使用固定随机种子 `20260716`，确保实验可重复。

| 系统 | 水平 RMSE | 垂直 RMSE | 三维 RMSE | 100 m RPE |
|---|---:|---:|---:|---:|
| 模拟厘米 RTK | 0.031 m | 0.040 m | 0.050 m | 0.043 m |
| **Pose3 + 模拟 RTK** | **0.066 m** | **0.339 m** | **0.345 m** | **0.043 m** |

<p align="center">
  <img src="docs/assets/urbannav_synthetic_rtk_trajectory.png" width="96%" alt="UrbanNav模拟厘米RTK轨迹对比">
</p>

该实验用于研究系统架构的性能上限，不能作为独立实测精度声明。实验说明
当前系统在理想 GNSS 条件下具备厘米级水平定位能力；剩余垂直误差主要反映
LIO 高度、IMU 预积分和 GNSS 高度模型之间仍存在不一致。

机器可读的实验摘要保存在
[`docs/benchmarks`](docs/benchmarks)。

## 代码结构

```text
.
├── data/                         # 本地数据集，不上传 Git
├── docs/
│   ├── assets/                   # 系统架构与实验图片
│   └── benchmarks/               # 精简的实验指标
└── src/
    ├── faster-lio/               # 激光惯性前端
    └── faster_lio_gnss/          # 在线 Pose3 融合与评估
```

融合包只保留三个主要 launch 入口：

| Launch 文件 | 用途 |
|---|---|
| `pose3_fusion_live.launch` | 接入真实传感器和外部启动的 Faster-LIO |
| `urbannav_final_test.launch` | UrbanNav 真实 F9P 完整基准测试 |
| `urbannav_synthetic_rtk_test.launch` | 厘米级 RTK 性能上限实验 |

## 环境依赖

- Ubuntu 20.04
- ROS Noetic
- 支持 C++17 的编译器
- Eigen3、PCL、yaml-cpp、glog
- GTSAM 4.x
- Python 3、NumPy、Matplotlib

## 编译

```bash
cd /path/to/fasterlio_multi
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

也可以只编译两个核心包：

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
  --pkg faster_lio faster_lio_gnss
```

## 接入真实传感器

首先启动激光雷达、IMU、GNSS 驱动和 Faster-LIO，然后运行：

```bash
roslaunch faster_lio_gnss pose3_fusion_live.launch
```

默认输入话题：

```text
/Odometry          nav_msgs/Odometry
/velodyne_points   sensor_msgs/PointCloud2
/imu/data          sensor_msgs/Imu
/gnss/fix          sensor_msgs/NavSatFix
```

主要输出：

```text
/odometry/pose3_online
/pose3_online/path/lio
/pose3_online/path/gnss
/pose3_online/path/fused
/pose3_online/loop_edges
TF: gnss_enu_pose3 -> odom
```

可以通过 launch 参数修改话题和 GNSS 天线杆臂：

```bash
roslaunch faster_lio_gnss pose3_fusion_live.launch \
  lio_topic:=/lio/odometry \
  cloud_topic:=/lidar/points \
  imu_topic:=/imu/data \
  gnss_topic:=/rtk/fix \
  antenna_x:=0.0 antenna_y:=-0.86 antenna_z:=0.31
```

### 实际接入要求

- LiDAR、IMU 和 GNSS 必须使用统一时间基准；
- 消息时间戳必须单调递增；
- IMU 单位、坐标轴方向和重力方向必须正确；
- `NavSatFix` 必须提供有效的状态和位置协方差；
- `antenna_x/y/z` 必须替换为实际 GNSS 天线杆臂；
- LiDAR 到 IMU 的外参必须经过标定；
- 不同传感器和环境需要重新调整 GNSS、IMU 和回环噪声。

## 复现 UrbanNav

按照 [`data/README.md`](data/README.md) 下载数据，然后运行：

```bash
roslaunch faster_lio_gnss urbannav_final_test.launch \
  bag_file:=/path/to/test.bag \
  nmea_file:=/path/to/f9p.splitter.nmea \
  ground_truth_file:=/path/to/UrbanNav_TST_GT_raw.txt
```

厘米级 RTK 实验：

```bash
roslaunch faster_lio_gnss urbannav_synthetic_rtk_test.launch \
  bag_file:=/path/to/test.bag \
  ground_truth_file:=/path/to/UrbanNav_TST_GT_raw.txt
```

默认评估结果保存在：

```text
~/.ros/faster_lio_gnss/
```

## 如何理解融合精度

融合系统并不保证在每一个时刻都比始终保持厘米精度的理想 RTK 更准确。
融合的主要工程意义是：

- 输出高频、连续的六自由度状态；
- 在 RTK 降级或短时失锁期间维持定位；
- 抑制城市多路径和非视距异常点；
- 同时提供位置、姿态、速度和地图坐标系一致性；
- GNSS 恢复后平滑修正累计漂移。

当前系统在 UrbanNav 差 GNSS 条件下达到约 `0.55 m` 水平 RMSE，在模拟
厘米 RTK 条件下达到约 `0.066 m` 水平 RMSE。

## 引用

本项目使用 Faster-LIO 作为激光惯性前端：

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

Faster-LIO 上游许可证位于
[`src/faster-lio/LICENSE`](src/faster-lio/LICENSE)。
