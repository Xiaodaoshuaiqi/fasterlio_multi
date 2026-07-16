#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthMetersPerDegree = 111320.0;

struct Date {
  int year = 0;
  int month = 0;
  int day = 0;

  bool Valid() const { return year > 1970 && month >= 1 && month <= 12 && day >= 1 && day <= 31; }
};

struct Motion {
  bool valid = false;
  double course_deg = 0.0;
  double speed_mps = 0.0;
};

struct Sample {
  ros::Time stamp;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  int quality = 0;
  int satellites = 0;
  double hdop = 0.0;
  bool heading_valid = false;
  double heading_enu = 0.0;
};

std::vector<std::string> Split(const std::string& input, char delimiter) {
  std::vector<std::string> fields;
  std::stringstream stream(input);
  std::string field;
  while (std::getline(stream, field, delimiter)) {
    fields.push_back(field);
  }
  if (!input.empty() && input.back() == delimiter) {
    fields.emplace_back();
  }
  return fields;
}

bool ParseDouble(const std::string& text, double& value) {
  if (text.empty()) {
    return false;
  }
  try {
    size_t parsed = 0;
    value = std::stod(text, &parsed);
    return parsed == text.size() && std::isfinite(value);
  } catch (const std::exception&) {
    return false;
  }
}

bool ParseInt(const std::string& text, int& value) {
  if (text.empty()) {
    return false;
  }
  try {
    size_t parsed = 0;
    value = std::stoi(text, &parsed);
    return parsed == text.size();
  } catch (const std::exception&) {
    return false;
  }
}

bool ExtractValidSentence(const std::string& raw_line, std::string& sentence) {
  const size_t dollar = raw_line.find('$');
  if (dollar == std::string::npos) {
    return false;
  }
  const size_t star = raw_line.find('*', dollar);
  if (star == std::string::npos || star + 2 >= raw_line.size()) {
    return false;
  }

  const std::string body = raw_line.substr(dollar + 1, star - dollar - 1);
  uint8_t checksum = 0;
  for (const unsigned char ch : body) {
    checksum ^= ch;
  }

  int expected = 0;
  try {
    expected = std::stoi(raw_line.substr(star + 1, 2), nullptr, 16);
  } catch (const std::exception&) {
    return false;
  }
  if (checksum != static_cast<uint8_t>(expected)) {
    return false;
  }

  sentence = "$" + body;
  return true;
}

bool EndsWith(const std::string& value, const std::string& suffix) {
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool ParseDateDdMmYy(const std::string& text, Date& date) {
  if (text.size() != 6) {
    return false;
  }
  int day = 0;
  int month = 0;
  int year = 0;
  if (!ParseInt(text.substr(0, 2), day) || !ParseInt(text.substr(2, 2), month) ||
      !ParseInt(text.substr(4, 2), year)) {
    return false;
  }
  date.day = day;
  date.month = month;
  date.year = year >= 80 ? 1900 + year : 2000 + year;
  return date.Valid();
}

bool ParseDateIso(const std::string& text, Date& date) {
  if (text.size() != 10 || text[4] != '-' || text[7] != '-') {
    return false;
  }
  return ParseInt(text.substr(0, 4), date.year) && ParseInt(text.substr(5, 2), date.month) &&
         ParseInt(text.substr(8, 2), date.day) && date.Valid();
}

bool ParseUtcSeconds(const std::string& text, double& seconds_of_day) {
  double hhmmss = 0.0;
  if (!ParseDouble(text, hhmmss)) {
    return false;
  }
  const int hour = static_cast<int>(hhmmss / 10000.0);
  const int minute = static_cast<int>((hhmmss - hour * 10000.0) / 100.0);
  const double second = hhmmss - hour * 10000.0 - minute * 100.0;
  if (hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0.0 || second >= 60.0) {
    return false;
  }
  seconds_of_day = hour * 3600.0 + minute * 60.0 + second;
  return true;
}

ros::Time MakeUtcStamp(const Date& date, double seconds_of_day, int day_offset) {
  std::tm utc{};
  utc.tm_year = date.year - 1900;
  utc.tm_mon = date.month - 1;
  utc.tm_mday = date.day + day_offset;
  utc.tm_isdst = 0;
  const std::time_t midnight = timegm(&utc);
  return ros::Time(static_cast<double>(midnight) + seconds_of_day);
}

bool ParseLatitudeLongitude(const std::string& value, const std::string& hemisphere, double& degrees) {
  double degrees_minutes = 0.0;
  if (!ParseDouble(value, degrees_minutes) || hemisphere.empty()) {
    return false;
  }
  const int whole_degrees = static_cast<int>(degrees_minutes / 100.0);
  const double minutes = degrees_minutes - whole_degrees * 100.0;
  degrees = whole_degrees + minutes / 60.0;
  if (hemisphere == "S" || hemisphere == "W") {
    degrees = -degrees;
  } else if (hemisphere != "N" && hemisphere != "E") {
    return false;
  }
  return std::isfinite(degrees);
}

double NormalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double HorizontalDistance(const Sample& first, const Sample& second, double& east, double& north) {
  const double mean_latitude = 0.5 * (first.latitude + second.latitude) * kPi / 180.0;
  east = (second.longitude - first.longitude) * kEarthMetersPerDegree * std::cos(mean_latitude);
  north = (second.latitude - first.latitude) * kEarthMetersPerDegree;
  return std::hypot(east, north);
}

}  // namespace

class NmeaReplayNode {
 public:
  NmeaReplayNode() : private_nh_("~") {
    private_nh_.param<std::string>("nmea_file", nmea_file_, "");
    private_nh_.param<std::string>("date_utc", date_utc_, "");
    private_nh_.param<std::string>("fix_topic", fix_topic_, "/gnss/fix");
    private_nh_.param<std::string>("heading_topic", heading_topic_, "/gnss/course_imu");
    private_nh_.param<std::string>("gps_frame", gps_frame_, "gps_link");
    private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
    private_nh_.param<int>("accepted_quality", accepted_quality_, 2);
    private_nh_.param<double>("east_variance", east_variance_, 16.0);
    private_nh_.param<double>("north_variance", north_variance_, 16.0);
    private_nh_.param<double>("up_variance", up_variance_, 100.0);
    private_nh_.param<double>("minimum_heading_speed", minimum_heading_speed_, 2.0);
    private_nh_.param<double>("minimum_heading_baseline", minimum_heading_baseline_, 15.0);
    private_nh_.param<double>("minimum_heading_interval", minimum_heading_interval_, 3.0);
    private_nh_.param<double>("maximum_heading_interval", maximum_heading_interval_, 10.0);
    private_nh_.param<double>("heading_std_degrees", heading_std_degrees_, 15.0);
    private_nh_.param<double>("publish_delay", publish_delay_, 0.2);
    private_nh_.param<double>("maximum_publish_lag", maximum_publish_lag_, 2.0);

    if (nmea_file_.empty()) {
      throw std::runtime_error("~nmea_file is required");
    }

    LoadSamples();
    fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(fix_topic_, 10);
    heading_pub_ = nh_.advertise<sensor_msgs::Imu>(heading_topic_, 10);
    timer_ = nh_.createTimer(ros::Duration(0.01), &NmeaReplayNode::OnTimer, this);

    const size_t heading_count =
        static_cast<size_t>(std::count_if(samples_.begin(), samples_.end(), [](const Sample& sample) {
          return sample.heading_valid;
        }));
    ROS_INFO_STREAM("Loaded " << samples_.size() << " valid GNSS fixes and " << heading_count
                              << " heading samples from " << nmea_file_);
  }

 private:
  void LoadSamples() {
    std::ifstream input(nmea_file_, std::ios::binary);
    if (!input.is_open()) {
      throw std::runtime_error("failed to open NMEA file: " + nmea_file_);
    }

    Date date;
    if (!date_utc_.empty() && !ParseDateIso(date_utc_, date)) {
      throw std::runtime_error("~date_utc must use YYYY-MM-DD");
    }

    Motion latest_motion;
    std::string raw_line;
    int day_offset = 0;
    double previous_seconds_of_day = -1.0;
    size_t invalid_checksum_count = 0;
    size_t rejected_quality_count = 0;

    while (std::getline(input, raw_line)) {
      std::string sentence;
      if (!ExtractValidSentence(raw_line, sentence)) {
        if (raw_line.find('$') != std::string::npos) {
          ++invalid_checksum_count;
        }
        continue;
      }

      const std::vector<std::string> fields = Split(sentence, ',');
      if (fields.empty()) {
        continue;
      }

      if (EndsWith(fields[0], "RMC")) {
        if (fields.size() > 9 && date_utc_.empty()) {
          ParseDateDdMmYy(fields[9], date);
        }
        if (fields.size() > 8) {
          double speed_knots = 0.0;
          double course_deg = 0.0;
          if (ParseDouble(fields[7], speed_knots) && ParseDouble(fields[8], course_deg)) {
            latest_motion.valid = true;
            latest_motion.speed_mps = speed_knots * 0.514444;
            latest_motion.course_deg = course_deg;
          }
        }
        continue;
      }

      if (EndsWith(fields[0], "VTG")) {
        if (fields.size() > 7) {
          double course_deg = 0.0;
          double speed_kmh = 0.0;
          if (ParseDouble(fields[1], course_deg) && ParseDouble(fields[7], speed_kmh)) {
            latest_motion.valid = true;
            latest_motion.speed_mps = speed_kmh / 3.6;
            latest_motion.course_deg = course_deg;
          }
        }
        continue;
      }

      if (!EndsWith(fields[0], "GGA") || fields.size() < 12 || !date.Valid()) {
        continue;
      }

      int quality = 0;
      if (!ParseInt(fields[6], quality) || quality != accepted_quality_) {
        ++rejected_quality_count;
        latest_motion = Motion{};
        continue;
      }

      double seconds_of_day = 0.0;
      double latitude = 0.0;
      double longitude = 0.0;
      double orthometric_height = 0.0;
      double geoid_separation = 0.0;
      int satellites = 0;
      double hdop = 0.0;
      if (!ParseUtcSeconds(fields[1], seconds_of_day) ||
          !ParseLatitudeLongitude(fields[2], fields[3], latitude) ||
          !ParseLatitudeLongitude(fields[4], fields[5], longitude) ||
          !ParseDouble(fields[9], orthometric_height) || !ParseDouble(fields[11], geoid_separation) ||
          !ParseInt(fields[7], satellites) || !ParseDouble(fields[8], hdop)) {
        latest_motion = Motion{};
        continue;
      }

      if (previous_seconds_of_day >= 0.0 && seconds_of_day + 12.0 * 3600.0 < previous_seconds_of_day) {
        ++day_offset;
      }
      previous_seconds_of_day = seconds_of_day;

      Sample sample;
      sample.stamp = MakeUtcStamp(date, seconds_of_day, day_offset);
      sample.latitude = latitude;
      sample.longitude = longitude;
      sample.altitude = orthometric_height + geoid_separation;
      sample.quality = quality;
      sample.satellites = satellites;
      sample.hdop = hdop;
      if (latest_motion.valid && latest_motion.speed_mps >= minimum_heading_speed_) {
        sample.heading_valid = true;
        sample.heading_enu =
            NormalizeAngle(kPi / 2.0 - latest_motion.course_deg * kPi / 180.0);
      }
      samples_.push_back(sample);
      latest_motion = Motion{};
    }

    if (samples_.empty()) {
      throw std::runtime_error("no valid GGA fixes found in: " + nmea_file_);
    }

    FillPositionDerivedHeadings();
    ROS_INFO_STREAM("NMEA filtering: invalid/non-NMEA lines with '$'=" << invalid_checksum_count
                                                                       << ", rejected quality="
                                                                       << rejected_quality_count);
  }

  void FillPositionDerivedHeadings() {
    for (size_t i = 1; i < samples_.size(); ++i) {
      if (samples_[i].heading_valid) {
        continue;
      }
      for (size_t j = i; j-- > 0;) {
        const double interval = (samples_[i].stamp - samples_[j].stamp).toSec();
        if (interval > maximum_heading_interval_) {
          break;
        }
        if (interval < minimum_heading_interval_) {
          continue;
        }
        double east = 0.0;
        double north = 0.0;
        const double baseline = HorizontalDistance(samples_[j], samples_[i], east, north);
        if (baseline >= minimum_heading_baseline_) {
          samples_[i].heading_valid = true;
          samples_[i].heading_enu = std::atan2(north, east);
          break;
        }
      }
    }
  }

  void OnTimer(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (now.isZero()) {
      return;
    }

    if (!clock_initialized_ || (!last_now_.isZero() && now + ros::Duration(0.1) < last_now_)) {
      const ros::Time earliest = now - ros::Duration(maximum_publish_lag_);
      cursor_ = static_cast<size_t>(
          std::lower_bound(samples_.begin(), samples_.end(), earliest,
                           [](const Sample& sample, const ros::Time& stamp) { return sample.stamp < stamp; }) -
          samples_.begin());
      clock_initialized_ = true;
    }
    last_now_ = now;

    const ros::Time publish_before = now - ros::Duration(publish_delay_) + ros::Duration(0.02);
    while (cursor_ < samples_.size() && samples_[cursor_].stamp <= publish_before) {
      const Sample& sample = samples_[cursor_++];
      if (now - sample.stamp > ros::Duration(maximum_publish_lag_)) {
        continue;
      }
      PublishFix(sample);
      if (sample.heading_valid) {
        PublishHeading(sample);
      }
    }
  }

  void PublishFix(const Sample& sample) {
    sensor_msgs::NavSatFix fix;
    fix.header.stamp = sample.stamp;
    fix.header.frame_id = gps_frame_;
    fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS |
                         sensor_msgs::NavSatStatus::SERVICE_GLONASS |
                         sensor_msgs::NavSatStatus::SERVICE_GALILEO |
                         sensor_msgs::NavSatStatus::SERVICE_COMPASS;
    fix.latitude = sample.latitude;
    fix.longitude = sample.longitude;
    fix.altitude = sample.altitude;
    fix.position_covariance.fill(0.0);
    fix.position_covariance[0] = east_variance_;
    fix.position_covariance[4] = north_variance_;
    fix.position_covariance[8] = up_variance_;
    fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    fix_pub_.publish(fix);
  }

  void PublishHeading(const Sample& sample) {
    sensor_msgs::Imu heading;
    heading.header.stamp = sample.stamp;
    heading.header.frame_id = base_frame_;
    heading.orientation.x = 0.0;
    heading.orientation.y = 0.0;
    heading.orientation.z = std::sin(sample.heading_enu / 2.0);
    heading.orientation.w = std::cos(sample.heading_enu / 2.0);
    heading.orientation_covariance.fill(0.0);
    heading.orientation_covariance[0] = 1e6;
    heading.orientation_covariance[4] = 1e6;
    const double heading_std = heading_std_degrees_ * kPi / 180.0;
    heading.orientation_covariance[8] = heading_std * heading_std;
    heading.angular_velocity_covariance.fill(0.0);
    heading.angular_velocity_covariance[0] = -1.0;
    heading.linear_acceleration_covariance.fill(0.0);
    heading.linear_acceleration_covariance[0] = -1.0;
    heading_pub_.publish(heading);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher fix_pub_;
  ros::Publisher heading_pub_;
  ros::Timer timer_;

  std::string nmea_file_;
  std::string date_utc_;
  std::string fix_topic_;
  std::string heading_topic_;
  std::string gps_frame_;
  std::string base_frame_;
  int accepted_quality_ = 2;
  double east_variance_ = 16.0;
  double north_variance_ = 16.0;
  double up_variance_ = 100.0;
  double minimum_heading_speed_ = 2.0;
  double minimum_heading_baseline_ = 15.0;
  double minimum_heading_interval_ = 3.0;
  double maximum_heading_interval_ = 10.0;
  double heading_std_degrees_ = 15.0;
  double publish_delay_ = 0.2;
  double maximum_publish_lag_ = 2.0;

  std::vector<Sample> samples_;
  size_t cursor_ = 0;
  bool clock_initialized_ = false;
  ros::Time last_now_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "nmea_replay");
  try {
    NmeaReplayNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM(error.what());
    return 1;
  }
  return 0;
}
