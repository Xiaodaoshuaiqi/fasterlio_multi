#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84F = 1.0 / 298.257223563;
constexpr double kWgs84E2 = kWgs84F * (2.0 - kWgs84F);

double NormalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

gtsam::Pose3 MessagePose(const geometry_msgs::Pose& pose) {
  return gtsam::Pose3(
      gtsam::Rot3::Quaternion(pose.orientation.w, pose.orientation.x,
                              pose.orientation.y, pose.orientation.z),
      gtsam::Point3(pose.position.x, pose.position.y, pose.position.z));
}

geometry_msgs::Quaternion MessageQuaternion(const gtsam::Rot3& rotation) {
  const gtsam::Quaternion source = rotation.toQuaternion();
  geometry_msgs::Quaternion result;
  result.x = source.x();
  result.y = source.y();
  result.z = source.z();
  result.w = source.w();
  return result;
}

std::array<double, 3> GeodeticToEcef(double latitude, double longitude,
                                     double height) {
  const double lat = latitude * kPi / 180.0;
  const double lon = longitude * kPi / 180.0;
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double prime_vertical =
      kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
  return {(prime_vertical + height) * cos_lat * std::cos(lon),
          (prime_vertical + height) * cos_lat * std::sin(lon),
          (prime_vertical * (1.0 - kWgs84E2) + height) * sin_lat};
}

struct LioPose {
  ros::Time stamp;
  gtsam::Pose3 pose;
};

struct ImuSample {
  ros::Time stamp;
  gtsam::Vector3 acceleration = gtsam::Vector3::Zero();
  gtsam::Vector3 angular_velocity = gtsam::Vector3::Zero();
};

struct GnssSample {
  ros::Time stamp;
  gtsam::Point3 enu = gtsam::Point3::Zero();
  double east_variance = 0.0;
  double north_variance = 0.0;
};

struct Keyframe {
  LioPose lio;
  GnssSample gnss;
  ros::Time cloud_stamp;
  gtsam::Pose3 initial_pose;
  gtsam::Vector3 initial_velocity = gtsam::Vector3::Zero();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  bool use_gnss = false;
  bool use_loop = false;
  double innovation = 0.0;
  double increment_innovation = 0.0;
  double lio_speed = 0.0;
  double gnss_speed = 0.0;
};

struct LoopEdge {
  std::size_t first = 0;
  std::size_t second = 0;
  double fitness = 0.0;
};

struct LoopCandidate {
  std::size_t first = 0;
  double distance = 0.0;
  double yaw_difference = 0.0;
};

class HorizontalGnssAntennaFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {
 public:
  HorizontalGnssAntennaFactor(gtsam::Key pose_key, gtsam::Key bias_key,
                              const gtsam::Point2& measurement,
                              const gtsam::Point3& lever_arm,
                              const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>(
            model, pose_key, bias_key),
        measurement_(measurement),
        lever_arm_(lever_arm) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose, const gtsam::Point3& bias,
      boost::optional<gtsam::Matrix&> pose_jacobian = boost::none,
      boost::optional<gtsam::Matrix&> bias_jacobian =
          boost::none) const override {
    gtsam::Matrix36 transform_jacobian;
    const gtsam::Point3 prediction =
        pose.transformFrom(lever_arm_, transform_jacobian) + bias;
    if (pose_jacobian) {
      *pose_jacobian = transform_jacobian.topRows<2>();
    }
    if (bias_jacobian) {
      gtsam::Matrix23 jacobian = gtsam::Matrix23::Zero();
      jacobian(0, 0) = 1.0;
      jacobian(1, 1) = 1.0;
      *bias_jacobian = jacobian;
    }
    return gtsam::Point2(prediction.x(), prediction.y()) - measurement_;
  }

 private:
  gtsam::Point2 measurement_;
  gtsam::Point3 lever_arm_;
};

class HeightGnssAntennaFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {
 public:
  HeightGnssAntennaFactor(gtsam::Key pose_key, gtsam::Key bias_key,
                          double measurement,
                          const gtsam::Point3& lever_arm,
                          const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>(
            model, pose_key, bias_key),
        measurement_(measurement),
        lever_arm_(lever_arm) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose, const gtsam::Point3& bias,
      boost::optional<gtsam::Matrix&> pose_jacobian = boost::none,
      boost::optional<gtsam::Matrix&> bias_jacobian =
          boost::none) const override {
    gtsam::Matrix36 transform_jacobian;
    const gtsam::Point3 prediction =
        pose.transformFrom(lever_arm_, transform_jacobian) + bias;
    if (pose_jacobian) {
      *pose_jacobian = transform_jacobian.row(2);
    }
    if (bias_jacobian) {
      gtsam::Matrix13 jacobian = gtsam::Matrix13::Zero();
      jacobian(0, 2) = 1.0;
      *bias_jacobian = jacobian;
    }
    return (gtsam::Vector1() << prediction.z() - measurement_).finished();
  }

 private:
  double measurement_;
  gtsam::Point3 lever_arm_;
};

}  // namespace

class OnlinePose3Fusion {
 public:
  OnlinePose3Fusion() : private_nh_("~") {
    LoadParameters();
    ConfigureNoiseModels();
    ResetIsam();

    lio_subscriber_ = nh_.subscribe<nav_msgs::Odometry>(
        lio_topic_, 5000, &OnlinePose3Fusion::OnLio, this);
    gnss_subscriber_ = nh_.subscribe<sensor_msgs::NavSatFix>(
        gnss_topic_, 500, &OnlinePose3Fusion::OnGnss, this);
    if (use_imu_) {
      imu_subscriber_ = nh_.subscribe<sensor_msgs::Imu>(
          imu_topic_, 200000, &OnlinePose3Fusion::OnImu, this);
    }
    if (use_loop_) {
      cloud_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(
          cloud_topic_, 100, &OnlinePose3Fusion::OnCloud, this);
    }

    fused_odometry_publisher_ =
        nh_.advertise<nav_msgs::Odometry>(fused_odometry_topic_, 100);
    lio_path_publisher_ =
        nh_.advertise<nav_msgs::Path>(lio_path_topic_, 1, true);
    gnss_path_publisher_ =
        nh_.advertise<nav_msgs::Path>(gnss_path_topic_, 1, true);
    fused_path_publisher_ =
        nh_.advertise<nav_msgs::Path>(fused_path_topic_, 1, true);
    datum_publisher_ =
        nh_.advertise<sensor_msgs::NavSatFix>(datum_topic_, 1, true);
    alignment_status_publisher_ =
        nh_.advertise<std_msgs::Bool>(alignment_status_topic_, 1, true);
    loop_marker_publisher_ =
        nh_.advertise<visualization_msgs::Marker>(loop_marker_topic_, 1,
                                                  true);

    lio_path_.header.frame_id = world_frame_;
    gnss_path_.header.frame_id = world_frame_;
    fused_path_.header.frame_id = world_frame_;
    PublishAlignmentStatus(false);
    PublishPaths();
    PublishLoopMarkers();

    ROS_INFO_STREAM("Online Pose3 fusion waits " << warmup_seconds_
                                                 << " seconds for alignment"
                                                 << " [height=" << use_height_
                                                 << ", imu=" << use_imu_
                                                 << ", loop=" << use_loop_
                                                 << "]");
  }

 private:
  void LoadParameters() {
    private_nh_.param<std::string>("lio_topic", lio_topic_, "/Odometry");
    private_nh_.param<std::string>("gnss_topic", gnss_topic_, "/gnss/fix");
    private_nh_.param<std::string>("imu_topic", imu_topic_, "/imu/data");
    private_nh_.param<std::string>("cloud_topic", cloud_topic_,
                                   "/velodyne_points");
    private_nh_.param<std::string>("fused_odometry_topic",
                                   fused_odometry_topic_,
                                   "/odometry/pose3_online");
    private_nh_.param<std::string>("lio_path_topic", lio_path_topic_,
                                   "/pose3_online/path/lio");
    private_nh_.param<std::string>("gnss_path_topic", gnss_path_topic_,
                                   "/pose3_online/path/gnss");
    private_nh_.param<std::string>("fused_path_topic", fused_path_topic_,
                                   "/pose3_online/path/fused");
    private_nh_.param<std::string>("datum_topic", datum_topic_,
                                   "/pose3_online/datum");
    private_nh_.param<std::string>("alignment_status_topic",
                                   alignment_status_topic_,
                                   "/pose3_online/alignment_ready");
    private_nh_.param<std::string>("loop_marker_topic", loop_marker_topic_,
                                   "/pose3_online/loop_edges");
    private_nh_.param<std::string>("world_frame", world_frame_,
                                   "gnss_enu_pose3");
    private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");

    private_nh_.param<bool>("use_height", use_height_, true);
    private_nh_.param<bool>("use_imu", use_imu_, true);
    private_nh_.param<bool>("use_loop", use_loop_, true);
    private_nh_.param<double>("warmup_seconds", warmup_seconds_, 60.0);
    private_nh_.param<double>("gnss_sigma_xy", gnss_sigma_xy_, 5.0);
    private_nh_.param<double>("gnss_sigma_z", gnss_sigma_z_, 2.0);
    private_nh_.param<double>("odom_sigma_xy", odom_sigma_xy_, 0.12);
    private_nh_.param<double>("odom_sigma_z", odom_sigma_z_, 0.20);
    private_nh_.param<double>("odom_sigma_roll_pitch_degrees",
                              odom_sigma_roll_pitch_degrees_, 0.30);
    private_nh_.param<double>("odom_sigma_yaw_degrees",
                              odom_sigma_yaw_degrees_, 0.06);
    private_nh_.param<double>("gnss_bias_rw_xy", gnss_bias_rw_xy_, 0.60);
    private_nh_.param<double>("gnss_bias_rw_z", gnss_bias_rw_z_, 0.30);
    private_nh_.param<double>("gnss_bias_prior_xy",
                              gnss_bias_prior_xy_, 4.0);
    private_nh_.param<double>("gnss_bias_prior_z",
                              gnss_bias_prior_z_, 8.0);
    private_nh_.param<double>("huber_k", huber_k_, 1.0);
    private_nh_.param<double>("maximum_speed", maximum_speed_, 25.0);
    private_nh_.param<double>("maximum_speed_difference",
                              maximum_speed_difference_, 6.0);
    private_nh_.param<double>("maximum_innovation",
                              maximum_innovation_, 15.0);
    private_nh_.param<double>("maximum_increment_innovation",
                              maximum_increment_innovation_, 5.0);
    private_nh_.param<double>("maximum_horizontal_variance",
                              maximum_horizontal_variance_, 100.0);
    private_nh_.param<int>("consecutive_fixes", consecutive_fixes_, 3);
    private_nh_.param<int>("path_publish_stride", path_publish_stride_, 10);
    private_nh_.param<double>("antenna_x", antenna_x_, 0.0);
    private_nh_.param<double>("antenna_y", antenna_y_, -0.86);
    private_nh_.param<double>("antenna_z", antenna_z_, 0.31);
    private_nh_.param<double>("lidar_to_imu_z", lidar_to_imu_z_, 0.28);

    private_nh_.param<double>("imu_accel_noise", imu_accel_noise_, 0.10);
    private_nh_.param<double>("imu_gyro_noise", imu_gyro_noise_, 0.01);
    private_nh_.param<double>("imu_accel_bias_rw",
                              imu_accel_bias_rw_, 0.01);
    private_nh_.param<double>("imu_gyro_bias_rw",
                              imu_gyro_bias_rw_, 0.001);
    private_nh_.param<double>("imu_lio_velocity_sigma_xy",
                              imu_lio_velocity_sigma_xy_, 0.0);
    private_nh_.param<double>("imu_lio_velocity_sigma_z",
                              imu_lio_velocity_sigma_z_, 1.0);

    private_nh_.param<double>("loop_candidate_radius",
                              loop_candidate_radius_, 7.0);
    private_nh_.param<double>("loop_minimum_separation",
                              loop_minimum_separation_, 120.0);
    private_nh_.param<double>("loop_spacing", loop_spacing_, 15.0);
    private_nh_.param<double>("loop_voxel_size", loop_voxel_size_, 0.80);
    private_nh_.param<double>("loop_max_correspondence",
                              loop_max_correspondence_, 6.0);
    private_nh_.param<double>("loop_max_fitness", loop_max_fitness_, 0.50);
    private_nh_.param<double>("loop_max_correction",
                              loop_max_correction_, 8.0);
    private_nh_.param<double>("loop_rotation_sigma_degrees",
                              loop_rotation_sigma_degrees_, 4.0);
    private_nh_.param<double>("loop_translation_sigma",
                              loop_translation_sigma_, 1.0);
    private_nh_.param<double>("loop_vertical_sigma",
                              loop_vertical_sigma_, 1.5);
    private_nh_.param<double>("loop_cloud_match_tolerance",
                              loop_cloud_match_tolerance_, 0.25);
    private_nh_.param<int>("loop_cloud_buffer_size",
                           loop_cloud_buffer_size_, 500);
  }

  void ConfigureNoiseModels() {
    const double roll_pitch_sigma =
        odom_sigma_roll_pitch_degrees_ * kPi / 180.0;
    const double yaw_sigma = odom_sigma_yaw_degrees_ * kPi / 180.0;
    odometry_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() << roll_pitch_sigma, roll_pitch_sigma, yaw_sigma,
         odom_sigma_xy_, odom_sigma_xy_, odom_sigma_z_)
            .finished());
    pose_prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() << 1.0 * kPi / 180.0, 1.0 * kPi / 180.0,
         3.0 * kPi / 180.0, 0.5, 0.5, 0.5)
            .finished());
    velocity_prior_noise_ =
        gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
    imu_bias_prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() << 0.10, 0.10, 0.10, 0.01, 0.01, 0.01)
            .finished());
    if (imu_lio_velocity_sigma_z_ > 0.0) {
      const double horizontal_sigma =
          imu_lio_velocity_sigma_xy_ > 0.0
              ? imu_lio_velocity_sigma_xy_
              : 1000.0;
      imu_lio_velocity_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector3()
               << horizontal_sigma, horizontal_sigma,
           imu_lio_velocity_sigma_z_)
              .finished());
    }
    gnss_bias_prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector3() << gnss_bias_prior_xy_, gnss_bias_prior_xy_,
         gnss_bias_prior_z_)
            .finished());

    const auto horizontal_gaussian =
        gtsam::noiseModel::Isotropic::Sigma(2, gnss_sigma_xy_);
    horizontal_gnss_noise_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(huber_k_),
        horizontal_gaussian);
    const auto height_gaussian =
        gtsam::noiseModel::Isotropic::Sigma(1, gnss_sigma_z_);
    height_gnss_noise_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(huber_k_),
        height_gaussian);

    const double loop_rotation_sigma =
        loop_rotation_sigma_degrees_ * kPi / 180.0;
    const auto loop_gaussian = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() << loop_rotation_sigma, loop_rotation_sigma,
         loop_rotation_sigma, loop_translation_sigma_,
         loop_translation_sigma_, loop_vertical_sigma_)
            .finished());
    loop_noise_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(2.0),
        loop_gaussian);

    imu_parameters_ = gtsam::PreintegrationParams::MakeSharedU(9.81);
    imu_parameters_->setAccelerometerCovariance(
        gtsam::Matrix3::Identity() * imu_accel_noise_ * imu_accel_noise_);
    imu_parameters_->setGyroscopeCovariance(
        gtsam::Matrix3::Identity() * imu_gyro_noise_ * imu_gyro_noise_);
    imu_parameters_->setIntegrationCovariance(
        gtsam::Matrix3::Identity() * 1e-6);
  }

  void ResetIsam() {
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam_ = std::make_unique<gtsam::ISAM2>(parameters);
    estimate_.clear();
  }

  void Reset() {
    lio_poses_.clear();
    imu_samples_.clear();
    pending_gnss_.clear();
    keyframes_.clear();
    loop_edges_.clear();
    cloud_messages_.clear();
    lio_path_.poses.clear();
    gnss_path_.poses.clear();
    fused_path_.poses.clear();
    datum_ready_ = false;
    alignment_ready_ = false;
    consecutive_valid_ = 0;
    accepted_gnss_count_ = 0;
    rejected_gnss_count_ = 0;
    path_publish_counter_ = 0;
    latest_correction_ = gtsam::Pose3();
    alignment_ = gtsam::Pose3();
    last_lio_stamp_ = ros::Time();
    last_gnss_stamp_ = ros::Time();
    last_imu_stamp_ = ros::Time();
    last_cloud_stamp_ = ros::Time();
    last_loop_stamp_ = ros::Time();
    ResetIsam();
    PublishAlignmentStatus(false);
    PublishPaths();
    PublishLoopMarkers();
    ROS_WARN("Detected time rewind; reset online Pose3 fusion");
  }

  void OnLio(const nav_msgs::Odometry::ConstPtr& message) {
    if (!last_lio_stamp_.isZero() &&
        message->header.stamp + ros::Duration(0.1) < last_lio_stamp_) {
      Reset();
    }
    LioPose pose;
    pose.stamp = message->header.stamp;
    pose.pose = MessagePose(message->pose.pose);
    last_lio_stamp_ = pose.stamp;
    lio_poses_.push_back(pose);

    const bool alignment_was_ready = alignment_ready_;
    ProcessPendingGnss();
    BroadcastWorldToOdom(pose.stamp);
    if (!alignment_ready_) {
      return;
    }

    const gtsam::Pose3 initial_global = alignment_.compose(pose.pose);
    if (alignment_was_ready) {
      AppendPose(lio_path_, pose.stamp, initial_global);
    }
    const gtsam::Pose3 fused =
        latest_correction_.compose(initial_global);
    PublishFusedOdometry(pose.stamp, fused);
    if (++path_publish_counter_ %
            static_cast<std::size_t>(std::max(path_publish_stride_, 1)) ==
        0) {
      PublishPaths();
    }
  }

  void OnImu(const sensor_msgs::Imu::ConstPtr& message) {
    if (!use_imu_) {
      return;
    }
    if (!last_imu_stamp_.isZero() &&
        message->header.stamp + ros::Duration(0.1) < last_imu_stamp_) {
      return;
    }
    ImuSample sample;
    sample.stamp = message->header.stamp;
    sample.acceleration =
        gtsam::Vector3(message->linear_acceleration.x,
                       message->linear_acceleration.y,
                       message->linear_acceleration.z);
    sample.angular_velocity =
        gtsam::Vector3(message->angular_velocity.x,
                       message->angular_velocity.y,
                       message->angular_velocity.z);
    imu_samples_.push_back(sample);
    last_imu_stamp_ = sample.stamp;
  }

  void OnCloud(const sensor_msgs::PointCloud2::ConstPtr& message) {
    if (!use_loop_) {
      return;
    }
    if (!last_cloud_stamp_.isZero() &&
        message->header.stamp + ros::Duration(0.1) <
            last_cloud_stamp_) {
      cloud_messages_.clear();
    }
    const auto insertion = std::upper_bound(
        cloud_messages_.begin(), cloud_messages_.end(),
        message->header.stamp,
        [](const ros::Time& stamp,
           const sensor_msgs::PointCloud2::ConstPtr& queued) {
          return stamp < queued->header.stamp;
        });
    cloud_messages_.insert(insertion, message);
    last_cloud_stamp_ = message->header.stamp;
    while (cloud_messages_.size() >
           static_cast<std::size_t>(
               std::max(loop_cloud_buffer_size_, 2))) {
      cloud_messages_.pop_front();
    }
    ProcessPendingGnss();
  }

  void OnGnss(const sensor_msgs::NavSatFix::ConstPtr& message) {
    if (message->status.status < sensor_msgs::NavSatStatus::STATUS_FIX ||
        !std::isfinite(message->latitude) ||
        !std::isfinite(message->longitude) ||
        !std::isfinite(message->altitude)) {
      ROS_WARN_THROTTLE(2.0, "Reject invalid GNSS fix");
      return;
    }
    const double east_variance = message->position_covariance[0];
    const double north_variance = message->position_covariance[4];
    const bool excessive_covariance =
        (east_variance > 0.0 &&
         east_variance > maximum_horizontal_variance_) ||
        (north_variance > 0.0 &&
         north_variance > maximum_horizontal_variance_);
    if (excessive_covariance) {
      ROS_WARN_THROTTLE(2.0, "Reject GNSS fix with excessive covariance");
      return;
    }
    if (!last_gnss_stamp_.isZero() &&
        message->header.stamp + ros::Duration(0.1) < last_gnss_stamp_) {
      return;
    }
    if (!datum_ready_) {
      SetDatum(*message);
    }
    GnssSample sample;
    sample.stamp = message->header.stamp;
    sample.enu = GeodeticToEnu(message->latitude, message->longitude,
                               message->altitude);
    sample.east_variance = east_variance;
    sample.north_variance = north_variance;
    last_gnss_stamp_ = sample.stamp;
    const auto insertion = std::upper_bound(
        pending_gnss_.begin(), pending_gnss_.end(), sample.stamp,
        [](const ros::Time& stamp, const GnssSample& queued) {
          return stamp < queued.stamp;
        });
    pending_gnss_.insert(insertion, sample);
    ProcessPendingGnss();
  }

  void SetDatum(const sensor_msgs::NavSatFix& fix) {
    datum_latitude_ = fix.latitude;
    datum_longitude_ = fix.longitude;
    datum_altitude_ = fix.altitude;
    datum_ecef_ =
        GeodeticToEcef(datum_latitude_, datum_longitude_, datum_altitude_);
    datum_ready_ = true;
    sensor_msgs::NavSatFix datum = fix;
    datum.header.stamp = ros::Time::now();
    datum.header.frame_id = world_frame_;
    datum_publisher_.publish(datum);
    ROS_INFO_STREAM("Set Pose3 ENU datum from first GNSS fix: "
                    << datum_latitude_ << ", " << datum_longitude_ << ", "
                    << datum_altitude_);
  }

  gtsam::Point3 GeodeticToEnu(double latitude, double longitude,
                              double altitude) const {
    const std::array<double, 3> point =
        GeodeticToEcef(latitude, longitude, altitude);
    const double dx = point[0] - datum_ecef_[0];
    const double dy = point[1] - datum_ecef_[1];
    const double dz = point[2] - datum_ecef_[2];
    const double lat = datum_latitude_ * kPi / 180.0;
    const double lon = datum_longitude_ * kPi / 180.0;
    return gtsam::Point3(
        -std::sin(lon) * dx + std::cos(lon) * dy,
        -std::sin(lat) * std::cos(lon) * dx -
            std::sin(lat) * std::sin(lon) * dy + std::cos(lat) * dz,
        std::cos(lat) * std::cos(lon) * dx +
            std::cos(lat) * std::sin(lon) * dy + std::sin(lat) * dz);
  }

  bool InterpolateLio(const ros::Time& stamp, LioPose& result) const {
    const auto upper = std::lower_bound(
        lio_poses_.begin(), lio_poses_.end(), stamp,
        [](const LioPose& pose, const ros::Time& query) {
          return pose.stamp < query;
        });
    if (upper == lio_poses_.end()) {
      return false;
    }
    if (upper->stamp == stamp) {
      result = *upper;
      return true;
    }
    if (upper == lio_poses_.begin()) {
      return false;
    }
    const LioPose& second = *upper;
    const LioPose& first = *(upper - 1);
    const double interval = (second.stamp - first.stamp).toSec();
    if (interval <= 0.0) {
      return false;
    }
    const double ratio = (stamp - first.stamp).toSec() / interval;
    result.stamp = stamp;
    const gtsam::Vector6 delta =
        gtsam::Pose3::Logmap(first.pose.between(second.pose));
    result.pose =
        first.pose.compose(gtsam::Pose3::Expmap(ratio * delta));
    return true;
  }

  bool CloudMatchWindowReady(const ros::Time& stamp) const {
    return !use_loop_ ||
           (!cloud_messages_.empty() &&
            cloud_messages_.back()->header.stamp >=
                stamp + ros::Duration(loop_cloud_match_tolerance_));
  }

  sensor_msgs::PointCloud2::ConstPtr FindNearestCloud(
      const ros::Time& stamp) const {
    if (cloud_messages_.empty()) {
      return sensor_msgs::PointCloud2::ConstPtr();
    }
    const auto upper = std::lower_bound(
        cloud_messages_.begin(), cloud_messages_.end(), stamp,
        [](const sensor_msgs::PointCloud2::ConstPtr& cloud,
           const ros::Time& query) {
          return cloud->header.stamp < query;
        });
    sensor_msgs::PointCloud2::ConstPtr nearest;
    if (upper != cloud_messages_.end()) {
      nearest = *upper;
    }
    if (upper != cloud_messages_.begin()) {
      const sensor_msgs::PointCloud2::ConstPtr previous = *(upper - 1);
      if (!nearest ||
          std::abs((previous->header.stamp - stamp).toSec()) <=
              std::abs((nearest->header.stamp - stamp).toSec())) {
        nearest = previous;
      }
    }
    if (!nearest ||
        std::abs((nearest->header.stamp - stamp).toSec()) >
            loop_cloud_match_tolerance_) {
      return sensor_msgs::PointCloud2::ConstPtr();
    }
    return nearest;
  }

  void PruneCloudBuffer(const ros::Time& processed_stamp) {
    const ros::Time keep_after =
        processed_stamp - ros::Duration(loop_cloud_match_tolerance_);
    while (cloud_messages_.size() > 1 &&
           cloud_messages_[1]->header.stamp < keep_after) {
      cloud_messages_.pop_front();
    }
  }

  void ProcessPendingGnss() {
    while (!pending_gnss_.empty()) {
      if (lio_poses_.empty()) {
        return;
      }
      const GnssSample& sample = pending_gnss_.front();
      if (sample.stamp < lio_poses_.front().stamp) {
        pending_gnss_.pop_front();
        continue;
      }
      if (sample.stamp > lio_poses_.back().stamp) {
        return;
      }
      if (!CloudMatchWindowReady(sample.stamp)) {
        return;
      }
      LioPose lio;
      if (!InterpolateLio(sample.stamp, lio)) {
        return;
      }
      Keyframe keyframe;
      keyframe.gnss = sample;
      keyframe.lio = lio;
      const sensor_msgs::PointCloud2::ConstPtr matched_cloud =
          FindNearestCloud(sample.stamp);
      if (matched_cloud) {
        keyframe.cloud_stamp = matched_cloud->header.stamp;
        keyframe.cloud = PrepareCloud(*matched_cloud);
      }
      keyframes_.push_back(keyframe);
      AppendGnssPose(keyframes_.back());
      if (!alignment_ready_) {
        if ((keyframes_.back().gnss.stamp -
             keyframes_.front().gnss.stamp)
                .toSec() >= warmup_seconds_) {
          InitializeGraph();
        }
      } else {
        AddIncrementalKeyframe(keyframes_.size() - 1);
      }
      PruneCloudBuffer(sample.stamp);
      pending_gnss_.pop_front();
    }
  }

  gtsam::Pose3 EstimateAlignment() const {
    std::vector<double> weights(keyframes_.size(), 1.0);
    double angle = 0.0;
    double tx = 0.0;
    double ty = 0.0;
    for (int iteration = 0; iteration < 8; ++iteration) {
      double weight_sum = 0.0;
      gtsam::Point2 lio_mean(0.0, 0.0);
      gtsam::Point2 gnss_mean(0.0, 0.0);
      for (std::size_t i = 0; i < keyframes_.size(); ++i) {
        weight_sum += weights[i];
        lio_mean +=
            weights[i] *
            gtsam::Point2(keyframes_[i].lio.pose.x(),
                          keyframes_[i].lio.pose.y());
        gnss_mean +=
            weights[i] *
            gtsam::Point2(keyframes_[i].gnss.enu.x(),
                          keyframes_[i].gnss.enu.y());
      }
      lio_mean /= weight_sum;
      gnss_mean /= weight_sum;
      double cross = 0.0;
      double dot = 0.0;
      for (std::size_t i = 0; i < keyframes_.size(); ++i) {
        const double lx = keyframes_[i].lio.pose.x() - lio_mean.x();
        const double ly = keyframes_[i].lio.pose.y() - lio_mean.y();
        const double gx = keyframes_[i].gnss.enu.x() - gnss_mean.x();
        const double gy = keyframes_[i].gnss.enu.y() - gnss_mean.y();
        cross += weights[i] * (lx * gy - ly * gx);
        dot += weights[i] * (lx * gx + ly * gy);
      }
      angle = std::atan2(cross, dot);
      const double cosine = std::cos(angle);
      const double sine = std::sin(angle);
      tx =
          gnss_mean.x() - (cosine * lio_mean.x() - sine * lio_mean.y());
      ty =
          gnss_mean.y() - (sine * lio_mean.x() + cosine * lio_mean.y());
      for (std::size_t i = 0; i < keyframes_.size(); ++i) {
        const double x = cosine * keyframes_[i].lio.pose.x() -
                         sine * keyframes_[i].lio.pose.y() + tx;
        const double y = sine * keyframes_[i].lio.pose.x() +
                         cosine * keyframes_[i].lio.pose.y() + ty;
        const double residual =
            std::hypot(x - keyframes_[i].gnss.enu.x(),
                       y - keyframes_[i].gnss.enu.y());
        constexpr double kHuberMeters = 4.0;
        weights[i] =
            residual <= kHuberMeters ? 1.0 : kHuberMeters / residual;
      }
    }
    std::vector<double> vertical_offsets;
    vertical_offsets.reserve(keyframes_.size());
    for (const Keyframe& keyframe : keyframes_) {
      vertical_offsets.push_back(keyframe.gnss.enu.z() -
                                 keyframe.lio.pose.z() - antenna_z_);
    }
    const auto middle =
        vertical_offsets.begin() + vertical_offsets.size() / 2;
    std::nth_element(vertical_offsets.begin(), middle,
                     vertical_offsets.end());
    return gtsam::Pose3(gtsam::Rot3::Rz(angle),
                        gtsam::Point3(tx, ty, *middle));
  }

  void InitializeVelocities() {
    for (std::size_t i = 0; i < keyframes_.size(); ++i) {
      const std::size_t first = i == 0 ? 0 : i - 1;
      const std::size_t second =
          i + 1 < keyframes_.size() ? i + 1 : keyframes_.size() - 1;
      const double dt = std::max(
          (keyframes_[second].gnss.stamp -
           keyframes_[first].gnss.stamp)
              .toSec(),
          1e-3);
      keyframes_[i].initial_velocity =
          (keyframes_[second].initial_pose.translation() -
           keyframes_[first].initial_pose.translation()) /
          dt;
      keyframes_[i].lio_speed =
          std::hypot(keyframes_[i].initial_velocity.x(),
                     keyframes_[i].initial_velocity.y());
    }
  }

  gtsam::imuBias::ConstantBias EstimateInitialImuBias() const {
    if (imu_samples_.empty() || keyframes_.empty()) {
      return gtsam::imuBias::ConstantBias();
    }
    const ros::Time start = keyframes_.front().gnss.stamp;
    gtsam::Vector3 acceleration = gtsam::Vector3::Zero();
    gtsam::Vector3 angular_velocity = gtsam::Vector3::Zero();
    std::size_t count = 0;
    for (const ImuSample& sample : imu_samples_) {
      if (sample.stamp < start) {
        continue;
      }
      if (sample.stamp > start + ros::Duration(5.0)) {
        break;
      }
      acceleration += sample.acceleration;
      angular_velocity += sample.angular_velocity;
      ++count;
    }
    if (count < 100) {
      return gtsam::imuBias::ConstantBias();
    }
    acceleration /= static_cast<double>(count);
    angular_velocity /= static_cast<double>(count);
    return gtsam::imuBias::ConstantBias(
        acceleration - gtsam::Vector3(0.0, 0.0, 9.81),
        angular_velocity);
  }

  gtsam::PreintegratedImuMeasurements Preintegrate(
      const ros::Time& start, const ros::Time& end,
      const gtsam::imuBias::ConstantBias& bias) const {
    gtsam::PreintegratedImuMeasurements preintegrated(imu_parameters_,
                                                       bias);
    auto current = std::upper_bound(
        imu_samples_.begin(), imu_samples_.end(), start,
        [](const ros::Time& stamp, const ImuSample& sample) {
          return stamp < sample.stamp;
        });
    if (current != imu_samples_.begin()) {
      --current;
    }
    for (; current + 1 != imu_samples_.end() && current->stamp < end;
         ++current) {
      const ros::Time interval_start =
          current->stamp < start ? start : current->stamp;
      const ros::Time interval_end =
          (current + 1)->stamp > end ? end : (current + 1)->stamp;
      const double dt = (interval_end - interval_start).toSec();
      if (dt > 0.0 && dt < 0.1) {
        preintegrated.integrateMeasurement(
            current->acceleration, current->angular_velocity, dt);
      }
    }
    return preintegrated;
  }

  void InitializeGraph() {
    alignment_ = EstimateAlignment();
    for (Keyframe& keyframe : keyframes_) {
      keyframe.initial_pose = alignment_.compose(keyframe.lio.pose);
    }
    InitializeVelocities();
    initial_imu_bias_ = EstimateInitialImuBias();
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    for (std::size_t i = 0; i < keyframes_.size(); ++i) {
      values.insert(PoseKey(i), keyframes_[i].initial_pose);
      values.insert(GnssBiasKey(i), gtsam::Point3(0.0, 0.0, 0.0));
      if (use_imu_) {
        values.insert(VelocityKey(i), keyframes_[i].initial_velocity);
        values.insert(ImuBiasKey(i), initial_imu_bias_);
        AddVelocityConsistencyFactor(i, graph);
      }
      if (i == 0) {
        graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            PoseKey(i), keyframes_[i].initial_pose, pose_prior_noise_);
        graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(
            GnssBiasKey(i), gtsam::Point3(0.0, 0.0, 0.0),
            gnss_bias_prior_noise_);
        if (use_imu_) {
          graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
              VelocityKey(i), keyframes_[i].initial_velocity,
              velocity_prior_noise_);
          graph.emplace_shared<
              gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
              ImuBiasKey(i), initial_imu_bias_, imu_bias_prior_noise_);
        }
      } else {
        AddMotionFactors(i, graph);
      }
    }
    isam_->update(graph, values);
    estimate_ = isam_->calculateEstimate();
    alignment_ready_ = true;
    latest_correction_ = gtsam::Pose3();
    PublishAlignmentStatus(true);
    BuildAlignedLioPath();
    RebuildFusedPath();
    BroadcastWorldToOdom(last_lio_stamp_);
    PublishPaths();
    ROS_INFO_STREAM("Initialized online Pose3 iSAM2 after "
                    << (keyframes_.back().gnss.stamp -
                        keyframes_.front().gnss.stamp)
                           .toSec()
                    << " seconds with " << keyframes_.size()
                    << " keyframes; warmup has no GNSS position factors");
  }

  void AddMotionFactors(std::size_t index,
                        gtsam::NonlinearFactorGraph& graph) const {
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        PoseKey(index - 1), PoseKey(index),
        keyframes_[index - 1].lio.pose.between(
            keyframes_[index].lio.pose),
        odometry_noise_);
    const double dt = std::max(
        (keyframes_[index].gnss.stamp -
         keyframes_[index - 1].gnss.stamp)
            .toSec(),
        1e-3);
    const auto gnss_bias_walk =
        gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector3()
                 << gnss_bias_rw_xy_ * std::sqrt(dt),
             gnss_bias_rw_xy_ * std::sqrt(dt),
             gnss_bias_rw_z_ * std::sqrt(dt))
                .finished());
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Point3>>(
        GnssBiasKey(index - 1), GnssBiasKey(index),
        gtsam::Point3(0.0, 0.0, 0.0), gnss_bias_walk);
    if (use_imu_) {
      const gtsam::imuBias::ConstantBias bias =
          estimate_.exists(ImuBiasKey(index - 1))
              ? estimate_.at<gtsam::imuBias::ConstantBias>(
                    ImuBiasKey(index - 1))
              : initial_imu_bias_;
      const gtsam::PreintegratedImuMeasurements preintegrated =
          Preintegrate(keyframes_[index - 1].gnss.stamp,
                       keyframes_[index].gnss.stamp, bias);
      graph.emplace_shared<gtsam::ImuFactor>(
          PoseKey(index - 1), VelocityKey(index - 1), PoseKey(index),
          VelocityKey(index), ImuBiasKey(index - 1), preintegrated);
      const auto imu_bias_walk =
          gtsam::noiseModel::Diagonal::Sigmas(
              (gtsam::Vector6()
                   << imu_accel_bias_rw_ * std::sqrt(dt),
               imu_accel_bias_rw_ * std::sqrt(dt),
               imu_accel_bias_rw_ * std::sqrt(dt),
               imu_gyro_bias_rw_ * std::sqrt(dt),
               imu_gyro_bias_rw_ * std::sqrt(dt),
               imu_gyro_bias_rw_ * std::sqrt(dt))
                  .finished());
      graph.emplace_shared<
          gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
          ImuBiasKey(index - 1), ImuBiasKey(index),
          gtsam::imuBias::ConstantBias(), imu_bias_walk);
    }
  }

  void AddVelocityConsistencyFactor(
      std::size_t index, gtsam::NonlinearFactorGraph& graph) const {
    if (!use_imu_ || !imu_lio_velocity_noise_) {
      return;
    }
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        VelocityKey(index), keyframes_[index].initial_velocity,
        imu_lio_velocity_noise_);
  }

  bool AcceptGnss(Keyframe& current) {
    const bool valid =
        current.gnss_speed <= maximum_speed_ &&
        std::abs(current.gnss_speed - current.lio_speed) <=
            maximum_speed_difference_ &&
        current.innovation <= maximum_innovation_ &&
        current.increment_innovation <= maximum_increment_innovation_;
    consecutive_valid_ = valid ? consecutive_valid_ + 1 : 0;
    return valid &&
           consecutive_valid_ >= std::max(consecutive_fixes_, 1);
  }

  void AddIncrementalKeyframe(std::size_t index) {
    Keyframe& current = keyframes_[index];
    const Keyframe& previous = keyframes_[index - 1];
    current.initial_pose = alignment_.compose(current.lio.pose);
    const double dt =
        std::max((current.gnss.stamp - previous.gnss.stamp).toSec(),
                 1e-3);
    current.initial_velocity =
        (current.initial_pose.translation() -
         previous.initial_pose.translation()) /
        dt;

    const gtsam::Pose3 relative =
        previous.lio.pose.between(current.lio.pose);
    const gtsam::Pose3 previous_optimized =
        estimate_.at<gtsam::Pose3>(PoseKey(index - 1));
    const gtsam::Pose3 predicted =
        previous_optimized.compose(relative);
    const gtsam::Point3 previous_bias =
        estimate_.at<gtsam::Point3>(GnssBiasKey(index - 1));
    const gtsam::Point3 antenna =
        predicted.transformFrom(
            gtsam::Point3(antenna_x_, antenna_y_, antenna_z_)) +
        previous_bias;
    current.innovation =
        std::hypot(antenna.x() - current.gnss.enu.x(),
                   antenna.y() - current.gnss.enu.y());
    const gtsam::Point3 gnss_delta =
        current.gnss.enu - previous.gnss.enu;
    const gtsam::Point3 lio_delta =
        current.initial_pose.translation() -
        previous.initial_pose.translation();
    current.increment_innovation =
        std::hypot(gnss_delta.x() - lio_delta.x(),
                   gnss_delta.y() - lio_delta.y());
    current.lio_speed =
        std::hypot(lio_delta.x(), lio_delta.y()) / dt;
    current.gnss_speed =
        std::hypot(gnss_delta.x(), gnss_delta.y()) / dt;
    current.use_gnss = AcceptGnss(current);

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    values.insert(PoseKey(index), predicted);
    values.insert(GnssBiasKey(index), previous_bias);
    if (use_imu_) {
      const gtsam::Vector3 previous_velocity =
          estimate_.at<gtsam::Vector3>(VelocityKey(index - 1));
      const gtsam::imuBias::ConstantBias previous_imu_bias =
          estimate_.at<gtsam::imuBias::ConstantBias>(
              ImuBiasKey(index - 1));
      values.insert(VelocityKey(index), previous_velocity);
      values.insert(ImuBiasKey(index), previous_imu_bias);
      AddVelocityConsistencyFactor(index, graph);
    }
    AddMotionFactors(index, graph);
    if (current.use_gnss) {
      const gtsam::Point3 lever(antenna_x_, antenna_y_, antenna_z_);
      graph.emplace_shared<HorizontalGnssAntennaFactor>(
          PoseKey(index), GnssBiasKey(index),
          gtsam::Point2(current.gnss.enu.x(), current.gnss.enu.y()),
          lever, horizontal_gnss_noise_);
      if (use_height_) {
        graph.emplace_shared<HeightGnssAntennaFactor>(
            PoseKey(index), GnssBiasKey(index), current.gnss.enu.z(),
            lever, height_gnss_noise_);
      }
      ++accepted_gnss_count_;
    } else {
      ++rejected_gnss_count_;
    }

    TryAddLoopFactor(index, graph);
    isam_->update(graph, values);
    estimate_ = isam_->calculateEstimate();
    const gtsam::Pose3 optimized =
        estimate_.at<gtsam::Pose3>(PoseKey(index));
    const gtsam::Point3 optimized_gnss_bias =
        estimate_.at<gtsam::Point3>(GnssBiasKey(index));
    latest_correction_ =
        optimized.compose(current.initial_pose.inverse());
    RebuildFusedPath();
    PublishPaths();
    PublishLoopMarkers();
    ROS_INFO_STREAM_THROTTLE(
        2.0, "Pose3 online: keyframes=" << keyframes_.size()
                                        << " GNSS accepted="
                                        << accepted_gnss_count_
                                        << " rejected="
                                        << rejected_gnss_count_
                                        << " loops="
                                        << loop_edges_.size()
                                        << " innovation="
                                        << current.innovation << " m"
                                        << " GNSS bias z="
                                        << optimized_gnss_bias.z() << " m");
    if (use_imu_) {
      const gtsam::Vector3 optimized_velocity =
          estimate_.at<gtsam::Vector3>(VelocityKey(index));
      const gtsam::imuBias::ConstantBias optimized_imu_bias =
          estimate_.at<gtsam::imuBias::ConstantBias>(
              ImuBiasKey(index));
      ROS_INFO_STREAM_THROTTLE(
          5.0, "Pose3 IMU vertical diagnostics: velocity="
                   << optimized_velocity.z() << " m/s, LIO velocity="
                   << current.initial_velocity.z()
                   << " m/s, accel bias z="
                   << optimized_imu_bias.accelerometer().z()
                   << " m/s^2");
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr PrepareCloud(
      const sensor_msgs::PointCloud2& message) const {
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(message, *raw);
    pcl::PointCloud<pcl::PointXYZI>::Ptr body(
        new pcl::PointCloud<pcl::PointXYZI>());
    body->reserve(raw->size());
    for (const pcl::PointXYZI& point : *raw) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }
      const double range =
          std::sqrt(point.x * point.x + point.y * point.y +
                    point.z * point.z);
      if (range < 4.0 || range > 70.0) {
        continue;
      }
      pcl::PointXYZI transformed = point;
      transformed.z += lidar_to_imu_z_;
      body->push_back(transformed);
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setLeafSize(loop_voxel_size_, loop_voxel_size_,
                      loop_voxel_size_);
    voxel.setInputCloud(body);
    voxel.filter(*filtered);
    return filtered;
  }

  std::vector<LoopCandidate> FindLoopCandidates(
      std::size_t second) const {
    std::vector<LoopCandidate> candidates;
    const gtsam::Pose3& second_pose = keyframes_[second].initial_pose;
    const double maximum_yaw = 60.0 * kPi / 180.0;
    for (std::size_t first = 0; first < second; ++first) {
      const double separation =
          (keyframes_[second].gnss.stamp -
           keyframes_[first].gnss.stamp)
              .toSec();
      if (separation < loop_minimum_separation_ ||
          !keyframes_[first].cloud ||
          keyframes_[first].lio_speed < 0.5 ||
          keyframes_[second].lio_speed < 0.5) {
        continue;
      }
      const gtsam::Pose3& first_pose = keyframes_[first].initial_pose;
      const gtsam::Point3 delta =
          second_pose.translation() - first_pose.translation();
      const double distance = std::hypot(delta.x(), delta.y());
      const double yaw_difference = std::abs(NormalizeAngle(
          second_pose.rotation().yaw() - first_pose.rotation().yaw()));
      if (distance <= loop_candidate_radius_ &&
          yaw_difference <= maximum_yaw) {
        candidates.push_back({first, distance, yaw_difference});
      }
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const LoopCandidate& left,
                 const LoopCandidate& right) {
                if (left.distance != right.distance) {
                  return left.distance < right.distance;
                }
                if (left.yaw_difference != right.yaw_difference) {
                  return left.yaw_difference < right.yaw_difference;
                }
                return left.first < right.first;
              });
    if (candidates.size() > 1U) {
      candidates.resize(1U);
    }
    return candidates;
  }

  void TryAddLoopFactor(std::size_t second,
                        gtsam::NonlinearFactorGraph& graph) {
    if (!use_loop_ || !keyframes_[second].cloud ||
        (!last_loop_stamp_.isZero() &&
         (keyframes_[second].gnss.stamp - last_loop_stamp_).toSec() <
             loop_spacing_)) {
      return;
    }
    const std::vector<LoopCandidate> candidates =
        FindLoopCandidates(second);
    if (candidates.empty()) {
      return;
    }
    const LoopCandidate& candidate = candidates.front();
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>
        gicp;
    gicp.setMaximumIterations(60);
    gicp.setMaxCorrespondenceDistance(loop_max_correspondence_);
    gicp.setTransformationEpsilon(1e-4);
    gicp.setEuclideanFitnessEpsilon(1e-4);
    gicp.setInputTarget(keyframes_[candidate.first].cloud);
    gicp.setInputSource(keyframes_[second].cloud);
    pcl::PointCloud<pcl::PointXYZI> aligned;
    const gtsam::Pose3 initial_relative =
        keyframes_[candidate.first].lio.pose.between(
            keyframes_[second].lio.pose);
    gicp.align(aligned, initial_relative.matrix().cast<float>());
    if (!gicp.hasConverged()) {
      return;
    }
    const double fitness = gicp.getFitnessScore(2.0);
    const gtsam::Pose3 measured(
        gicp.getFinalTransformation().cast<double>());
    const gtsam::Vector6 correction =
        gtsam::Pose3::Logmap(initial_relative.between(measured));
    if (!std::isfinite(fitness) || fitness > loop_max_fitness_ ||
        correction.tail<3>().norm() > loop_max_correction_ ||
        correction.head<3>().norm() > 30.0 * kPi / 180.0) {
      return;
    }
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        PoseKey(candidate.first), PoseKey(second), measured, loop_noise_);
    keyframes_[second].use_loop = true;
    loop_edges_.push_back({candidate.first, second, fitness});
    last_loop_stamp_ = keyframes_[second].gnss.stamp;
    ROS_INFO_STREAM("Accepted online GICP loop "
                    << candidate.first << " -> " << second
                    << " fitness=" << fitness << " cloud_dt=["
                    << (keyframes_[candidate.first].cloud_stamp -
                        keyframes_[candidate.first].gnss.stamp)
                           .toSec()
                    << ", "
                    << (keyframes_[second].cloud_stamp -
                        keyframes_[second].gnss.stamp)
                           .toSec()
                    << "] s");
  }

  void BuildAlignedLioPath() {
    lio_path_.poses.clear();
    for (const LioPose& pose : lio_poses_) {
      AppendPose(lio_path_, pose.stamp, alignment_.compose(pose.pose));
    }
  }

  void RebuildFusedPath() {
    fused_path_.poses.clear();
    for (std::size_t i = 0; i < keyframes_.size(); ++i) {
      if (estimate_.exists(PoseKey(i))) {
        AppendPose(fused_path_, keyframes_[i].gnss.stamp,
                   estimate_.at<gtsam::Pose3>(PoseKey(i)));
      }
    }
  }

  void AppendGnssPose(const Keyframe& keyframe) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = world_frame_;
    pose.header.stamp = keyframe.gnss.stamp;
    pose.pose.position.x = keyframe.gnss.enu.x();
    pose.pose.position.y = keyframe.gnss.enu.y();
    pose.pose.position.z = keyframe.gnss.enu.z();
    pose.pose.orientation.w = 1.0;
    gnss_path_.header = pose.header;
    gnss_path_.poses.push_back(pose);
  }

  void AppendPose(nav_msgs::Path& path, const ros::Time& stamp,
                  const gtsam::Pose3& pose) const {
    geometry_msgs::PoseStamped message;
    message.header.frame_id = world_frame_;
    message.header.stamp = stamp;
    message.pose.position.x = pose.x();
    message.pose.position.y = pose.y();
    message.pose.position.z = pose.z();
    message.pose.orientation = MessageQuaternion(pose.rotation());
    path.header = message.header;
    path.poses.push_back(message);
  }

  void PublishFusedOdometry(const ros::Time& stamp,
                            const gtsam::Pose3& pose) {
    nav_msgs::Odometry message;
    message.header.frame_id = world_frame_;
    message.header.stamp = stamp;
    message.child_frame_id = "pose3_fused_imu_link";
    message.pose.pose.position.x = pose.x();
    message.pose.pose.position.y = pose.y();
    message.pose.pose.position.z = pose.z();
    message.pose.pose.orientation = MessageQuaternion(pose.rotation());
    fused_odometry_publisher_.publish(message);
  }

  void PublishAlignmentStatus(bool ready) {
    std_msgs::Bool message;
    message.data = ready;
    alignment_status_publisher_.publish(message);
  }

  void PublishPaths() {
    const ros::Time now = ros::Time::now();
    lio_path_.header.frame_id = world_frame_;
    lio_path_.header.stamp = now;
    gnss_path_.header.frame_id = world_frame_;
    gnss_path_.header.stamp = now;
    fused_path_.header.frame_id = world_frame_;
    fused_path_.header.stamp = now;
    lio_path_publisher_.publish(lio_path_);
    gnss_path_publisher_.publish(gnss_path_);
    fused_path_publisher_.publish(fused_path_);
  }

  void PublishLoopMarkers() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "pose3_loop_edges";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.35;
    marker.color.r = 1.0;
    marker.color.g = 0.75;
    marker.color.b = 0.1;
    marker.color.a = 0.9;
    for (const LoopEdge& edge : loop_edges_) {
      if (!estimate_.exists(PoseKey(edge.first)) ||
          !estimate_.exists(PoseKey(edge.second))) {
        continue;
      }
      const gtsam::Point3 first =
          estimate_.at<gtsam::Pose3>(PoseKey(edge.first)).translation();
      const gtsam::Point3 second =
          estimate_.at<gtsam::Pose3>(PoseKey(edge.second)).translation();
      geometry_msgs::Point first_message;
      first_message.x = first.x();
      first_message.y = first.y();
      first_message.z = first.z();
      geometry_msgs::Point second_message;
      second_message.x = second.x();
      second_message.y = second.y();
      second_message.z = second.z();
      marker.points.push_back(first_message);
      marker.points.push_back(second_message);
    }
    loop_marker_publisher_.publish(marker);
  }

  void BroadcastWorldToOdom(const ros::Time& stamp) {
    const gtsam::Pose3 world_from_odom =
        alignment_ready_ ? latest_correction_.compose(alignment_)
                         : gtsam::Pose3();
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = world_frame_;
    transform.header.stamp = stamp;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = world_from_odom.x();
    transform.transform.translation.y = world_from_odom.y();
    transform.transform.translation.z = world_from_odom.z();
    transform.transform.rotation =
        MessageQuaternion(world_from_odom.rotation());
    transform_broadcaster_.sendTransform(transform);
  }

  static gtsam::Key PoseKey(std::size_t index) {
    return gtsam::Symbol('x', index);
  }
  static gtsam::Key VelocityKey(std::size_t index) {
    return gtsam::Symbol('v', index);
  }
  static gtsam::Key ImuBiasKey(std::size_t index) {
    return gtsam::Symbol('i', index);
  }
  static gtsam::Key GnssBiasKey(std::size_t index) {
    return gtsam::Symbol('g', index);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber lio_subscriber_;
  ros::Subscriber gnss_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber cloud_subscriber_;
  ros::Publisher fused_odometry_publisher_;
  ros::Publisher lio_path_publisher_;
  ros::Publisher gnss_path_publisher_;
  ros::Publisher fused_path_publisher_;
  ros::Publisher datum_publisher_;
  ros::Publisher alignment_status_publisher_;
  ros::Publisher loop_marker_publisher_;
  tf2_ros::TransformBroadcaster transform_broadcaster_;

  std::string lio_topic_;
  std::string gnss_topic_;
  std::string imu_topic_;
  std::string cloud_topic_;
  std::string fused_odometry_topic_;
  std::string lio_path_topic_;
  std::string gnss_path_topic_;
  std::string fused_path_topic_;
  std::string datum_topic_;
  std::string alignment_status_topic_;
  std::string loop_marker_topic_;
  std::string world_frame_;
  std::string odom_frame_;

  bool use_height_ = true;
  bool use_imu_ = true;
  bool use_loop_ = true;
  double warmup_seconds_ = 60.0;
  double gnss_sigma_xy_ = 5.0;
  double gnss_sigma_z_ = 2.0;
  double odom_sigma_xy_ = 0.12;
  double odom_sigma_z_ = 0.20;
  double odom_sigma_roll_pitch_degrees_ = 0.30;
  double odom_sigma_yaw_degrees_ = 0.06;
  double gnss_bias_rw_xy_ = 0.60;
  double gnss_bias_rw_z_ = 0.30;
  double gnss_bias_prior_xy_ = 4.0;
  double gnss_bias_prior_z_ = 8.0;
  double huber_k_ = 1.0;
  double maximum_speed_ = 25.0;
  double maximum_speed_difference_ = 6.0;
  double maximum_innovation_ = 15.0;
  double maximum_increment_innovation_ = 5.0;
  double maximum_horizontal_variance_ = 100.0;
  int consecutive_fixes_ = 3;
  int path_publish_stride_ = 10;
  double antenna_x_ = 0.0;
  double antenna_y_ = -0.86;
  double antenna_z_ = 0.31;
  double lidar_to_imu_z_ = 0.28;
  double imu_accel_noise_ = 0.10;
  double imu_gyro_noise_ = 0.01;
  double imu_accel_bias_rw_ = 0.01;
  double imu_gyro_bias_rw_ = 0.001;
  double imu_lio_velocity_sigma_xy_ = 0.0;
  double imu_lio_velocity_sigma_z_ = 1.0;
  double loop_candidate_radius_ = 7.0;
  double loop_minimum_separation_ = 120.0;
  double loop_spacing_ = 15.0;
  double loop_voxel_size_ = 0.80;
  double loop_max_correspondence_ = 6.0;
  double loop_max_fitness_ = 0.50;
  double loop_max_correction_ = 8.0;
  double loop_rotation_sigma_degrees_ = 4.0;
  double loop_translation_sigma_ = 1.0;
  double loop_vertical_sigma_ = 1.5;
  double loop_cloud_match_tolerance_ = 0.25;
  int loop_cloud_buffer_size_ = 500;

  gtsam::SharedNoiseModel odometry_noise_;
  gtsam::SharedNoiseModel pose_prior_noise_;
  gtsam::SharedNoiseModel velocity_prior_noise_;
  gtsam::SharedNoiseModel imu_bias_prior_noise_;
  gtsam::SharedNoiseModel imu_lio_velocity_noise_;
  gtsam::SharedNoiseModel gnss_bias_prior_noise_;
  gtsam::SharedNoiseModel horizontal_gnss_noise_;
  gtsam::SharedNoiseModel height_gnss_noise_;
  gtsam::SharedNoiseModel loop_noise_;
  boost::shared_ptr<gtsam::PreintegrationParams> imu_parameters_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::Values estimate_;

  std::vector<LioPose> lio_poses_;
  std::vector<ImuSample> imu_samples_;
  std::deque<GnssSample> pending_gnss_;
  std::deque<sensor_msgs::PointCloud2::ConstPtr> cloud_messages_;
  std::vector<Keyframe> keyframes_;
  std::vector<LoopEdge> loop_edges_;
  nav_msgs::Path lio_path_;
  nav_msgs::Path gnss_path_;
  nav_msgs::Path fused_path_;

  bool datum_ready_ = false;
  bool alignment_ready_ = false;
  double datum_latitude_ = 0.0;
  double datum_longitude_ = 0.0;
  double datum_altitude_ = 0.0;
  std::array<double, 3> datum_ecef_{};
  gtsam::Pose3 alignment_;
  gtsam::Pose3 latest_correction_;
  gtsam::imuBias::ConstantBias initial_imu_bias_;
  ros::Time last_lio_stamp_;
  ros::Time last_gnss_stamp_;
  ros::Time last_imu_stamp_;
  ros::Time last_cloud_stamp_;
  ros::Time last_loop_stamp_;
  int consecutive_valid_ = 0;
  std::size_t accepted_gnss_count_ = 0;
  std::size_t rejected_gnss_count_ = 0;
  std::size_t path_publish_counter_ = 0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "online_pose3_fusion");
  OnlinePose3Fusion node;
  ros::spin();
  return 0;
}
