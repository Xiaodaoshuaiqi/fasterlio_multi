#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool

evaluation_script = Path(__file__).with_name("geodesy.py")
evaluation_spec = importlib.util.spec_from_file_location(
    "faster_lio_gnss_evaluation", evaluation_script
)
evaluation = importlib.util.module_from_spec(evaluation_spec)
evaluation_spec.loader.exec_module(evaluation)


def dms_to_degrees(degrees, minutes, seconds):
    sign = -1.0 if degrees < 0.0 else 1.0
    return degrees + sign * (abs(minutes) / 60.0 + abs(seconds) / 3600.0)


def load_samples(path):
    samples = []
    with open(path, encoding="ascii") as stream:
        for line in stream:
            fields = line.split()
            try:
                stamp = float(fields[0])
                latitude = dms_to_degrees(
                    float(fields[3]), float(fields[4]), float(fields[5])
                )
                longitude = dms_to_degrees(
                    float(fields[6]), float(fields[7]), float(fields[8])
                )
                altitude = float(fields[9])
            except (ValueError, IndexError):
                continue
            samples.append((stamp, latitude, longitude, altitude))
    if not samples:
        raise RuntimeError(f"no ground-truth samples in {path}")
    return samples


class OnlineGroundTruthReplay:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "gnss_enu")
        self.samples = load_samples(rospy.get_param("~ground_truth_file"))
        self.publisher = rospy.Publisher(
            rospy.get_param(
                "~path_topic", "/robust_online/path/ground_truth"
            ),
            RosPath,
            queue_size=1,
            latch=True,
        )
        self.path = RosPath()
        self.path.header.frame_id = self.frame_id
        self.reference = None
        self.cursor = 0
        self.last_now = rospy.Time()
        self.alignment_ready = False
        rospy.Subscriber(
            rospy.get_param("~datum_topic", "/robust_online/datum"),
            NavSatFix,
            self.on_datum,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~alignment_status_topic",
                "/robust_online/alignment_ready",
            ),
            Bool,
            self.on_alignment_status,
            queue_size=1,
        )
        self.timer = rospy.Timer(rospy.Duration(0.02), self.on_timer)

    def on_datum(self, message):
        self.reference = (
            message.latitude,
            message.longitude,
            message.altitude,
        )
        rospy.loginfo("Ground truth received online GNSS ENU datum")

    def on_alignment_status(self, message):
        self.alignment_ready = message.data
        if self.alignment_ready:
            if self.path.poses:
                self.publisher.publish(self.path)
            rospy.loginfo("Ground truth display enabled after online alignment")
        else:
            hidden = RosPath()
            hidden.header.frame_id = self.frame_id
            hidden.header.stamp = rospy.Time.now()
            self.publisher.publish(hidden)

    def reset(self):
        self.cursor = 0
        self.path.poses.clear()
        self.path.header.frame_id = self.frame_id
        self.path.header.stamp = rospy.Time.now()
        self.publisher.publish(self.path)

    def on_timer(self, _event):
        now = rospy.Time.now()
        if now.is_zero() or self.reference is None:
            return
        if not self.last_now.is_zero() and now + rospy.Duration(
            0.1
        ) < self.last_now:
            self.reset()
        self.last_now = now

        updated = False
        while (
            self.cursor < len(self.samples)
            and self.samples[self.cursor][0] <= now.to_sec()
        ):
            stamp, latitude, longitude, altitude = self.samples[self.cursor]
            enu = evaluation.geodetic_to_enu(
                latitude, longitude, altitude, self.reference
            )
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = rospy.Time.from_sec(stamp)
            pose.pose.position.x = float(enu[0])
            pose.pose.position.y = float(enu[1])
            pose.pose.position.z = float(enu[2])
            pose.pose.orientation.w = 1.0
            self.path.header = pose.header
            self.path.poses.append(pose)
            self.cursor += 1
            updated = True
        if self.alignment_ready and updated:
            self.publisher.publish(self.path)


def main():
    rospy.init_node("online_ground_truth_replay")
    OnlineGroundTruthReplay()
    rospy.spin()


if __name__ == "__main__":
    main()
