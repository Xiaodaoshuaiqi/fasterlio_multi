cmake_minimum_required(VERSION 3.0.2)
project(faster_lio_gnss)

add_compile_options(-std=c++17 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  visualization_msgs
)
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_conversions pcl_ros rosbag roscpp
                 rospy sensor_msgs std_msgs std_srvs tf2_ros visualization_msgs
  DEPENDS PCL
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${GTSAM_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(nmea_replay_node src/nmea_replay_node.cpp)
target_link_libraries(nmea_replay_node ${catkin_LIBRARIES})

add_executable(online_pose3_fusion_node src/online_pose3_fusion_node.cpp)
target_link_libraries(online_pose3_fusion_node
  gtsam ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS nmea_replay_node online_pose3_fusion_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/geodesy.py
  scripts/trajectory_metrics.py
  scripts/online_ground_truth_replay.py
  scripts/online_pose3_result_evaluator.py
  scripts/evaluation_aligned_path_replay.py
  scripts/synthetic_rtk_replay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
