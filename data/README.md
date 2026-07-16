# Dataset Directory

This directory is intentionally excluded from version control. Download the
UrbanNav files separately and place them here, or pass absolute paths through
the benchmark launch arguments.

The validated experiment uses:

```text
UrbanNav-HK-Medium-Urban-1.ublox.f9p.splitter.nmea
UrbanNav_TST_GT_raw.txt
```

The ROS bag is not stored in this repository. The default launch configuration
looks for `~/bags/test.bag`, but `bag_file`, `nmea_file`, and
`ground_truth_file` can all be overridden:

```bash
roslaunch faster_lio_gnss urbannav_final_test.launch \
  bag_file:=/path/to/urbannav.bag \
  nmea_file:=/path/to/f9p.splitter.nmea \
  ground_truth_file:=/path/to/UrbanNav_TST_GT_raw.txt
```

Dataset source: [UrbanNavDataset](https://github.com/IPNL-POLYU/UrbanNavDataset)
