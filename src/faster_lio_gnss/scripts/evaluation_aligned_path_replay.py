#!/usr/bin/env python3

import importlib.util
import math
import threading
from pathlib import Path

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path as RosPath


metrics_script = Path(__file__).with_name("trajectory_metrics.py")
metrics_spec = importlib.util.spec_from_file_location(
    "faster_lio_gnss_alignment_metrics", metrics_script
)
metrics = importlib.util.module_from_spec(metrics_spec)
metrics_spec.loader.exec_module(metrics)


def path_to_trajectory(message):
    rows = [
        (
            pose.header.stamp.to_sec(),
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )
        for pose in message.poses
    ]
    if not rows:
        return np.empty((0, 4))

    rows.sort(key=lambda row: row[0])
    unique = []
    for row in rows:
        if unique and abs(row[0] - unique[-1][0]) < 1e-6:
            unique[-1] = row
        else:
            unique.append(row)
    return np.asarray(unique, dtype=float)


def trajectory_to_path(trajectory, frame_id):
    message = RosPath()
    message.header.frame_id = frame_id
    message.header.stamp = rospy.Time.now()
    for row in trajectory:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.from_sec(float(row[0]))
        pose.pose.position.x = float(row[1])
        pose.pose.position.y = float(row[2])
        pose.pose.position.z = float(row[3])
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


class EvaluationAlignedPathReplay:
    def __init__(self):
        self.world_frame = rospy.get_param(
            "~world_frame", "gnss_enu_pose3"
        )
        self.evaluation_frame = rospy.get_param(
            "~evaluation_frame", "evaluation_aligned"
        )
        self.alignment_duration = float(
            rospy.get_param("~alignment_duration", 60.0)
        )
        self.publish_period = float(rospy.get_param("~publish_period", 1.0))
        self.minimum_samples = int(rospy.get_param("~minimum_samples", 20))
        self.lock = threading.Lock()
        self.messages = {}
        self.last_alignment = {}
        self.last_ros_time = rospy.Time()
        self.fused_transform = None
        self.transform_broadcaster = tf2_ros.TransformBroadcaster()

        paths = {
            "lio": (
                rospy.get_param(
                    "~lio_input_topic", "/pose3_online/path/lio"
                ),
                rospy.get_param(
                    "~lio_output_topic",
                    "/pose3_online/path/evaluation_aligned/lio",
                ),
            ),
            "gnss": (
                rospy.get_param(
                    "~gnss_input_topic", "/pose3_online/path/gnss"
                ),
                rospy.get_param(
                    "~gnss_output_topic",
                    "/pose3_online/path/evaluation_aligned/gnss",
                ),
            ),
            "fused": (
                rospy.get_param(
                    "~fused_input_topic", "/pose3_online/path/fused"
                ),
                rospy.get_param(
                    "~fused_output_topic",
                    "/pose3_online/path/evaluation_aligned/fused",
                ),
            ),
        }
        self.truth_topic = rospy.get_param(
            "~truth_input_topic", "/pose3_online/path/ground_truth"
        )
        self.truth_publisher = rospy.Publisher(
            rospy.get_param(
                "~truth_output_topic",
                "/pose3_online/path/evaluation_aligned/ground_truth",
            ),
            RosPath,
            queue_size=1,
            latch=True,
        )
        self.publishers = {}
        for name, (input_topic, output_topic) in paths.items():
            self.publishers[name] = rospy.Publisher(
                output_topic, RosPath, queue_size=1, latch=True
            )
            rospy.Subscriber(
                input_topic,
                RosPath,
                self.on_path,
                callback_args=name,
                queue_size=1,
            )
        rospy.Subscriber(
            self.truth_topic,
            RosPath,
            self.on_path,
            callback_args="truth",
            queue_size=1,
        )
        self.timer = rospy.Timer(
            rospy.Duration(max(self.publish_period, 0.1)),
            self.on_timer,
        )
        rospy.logwarn(
            "Evaluation-aligned RViz paths use ground truth for one fixed "
            "%.1f s alignment and must not be treated as deployable output",
            self.alignment_duration,
        )

    def on_path(self, message, name):
        with self.lock:
            self.messages[name] = message

    def snapshot(self):
        with self.lock:
            return dict(self.messages)

    def on_timer(self, _event):
        now = rospy.Time.now()
        if (
            not self.last_ros_time.is_zero()
            and now + rospy.Duration(0.1) < self.last_ros_time
        ):
            with self.lock:
                self.messages.clear()
            self.last_alignment.clear()
            self.fused_transform = None
            for publisher in self.publishers.values():
                publisher.publish(
                    trajectory_to_path(
                        np.empty((0, 4)), self.evaluation_frame
                    )
                )
            self.truth_publisher.publish(
                trajectory_to_path(
                    np.empty((0, 4)), self.evaluation_frame
                )
            )
            rospy.loginfo("Reset evaluation-aligned RViz paths after time rewind")
        self.last_ros_time = now

        messages = self.snapshot()
        if "truth" not in messages:
            self.publish_transform()
            return
        truth = path_to_trajectory(messages["truth"])
        if (
            len(truth) < self.minimum_samples
            or truth[-1, 0] - truth[0, 0] < self.alignment_duration
        ):
            self.publish_transform()
            return
        self.truth_publisher.publish(
            trajectory_to_path(truth, self.evaluation_frame)
        )

        for name in ("lio", "gnss", "fused"):
            if name not in messages:
                continue
            trajectory = path_to_trajectory(messages[name])
            if len(trajectory) < self.minimum_samples:
                continue
            try:
                estimate, matched_truth = metrics.interpolate_to_truth(
                    trajectory, truth
                )
                if (
                    len(matched_truth) < self.minimum_samples
                    or matched_truth[-1, 0] - matched_truth[0, 0]
                    < self.alignment_duration
                ):
                    continue
                angle, translation = metrics.robust_xy_alignment(
                    estimate, matched_truth, self.alignment_duration
                )
                rotation = np.array(
                    [
                        [math.cos(angle), -math.sin(angle)],
                        [math.sin(angle), math.cos(angle)],
                    ]
                )
                aligned = estimate.copy()
                aligned[:, 1:3] = (
                    estimate[:, 1:3] @ rotation.T + translation
                )
                vertical_mask = (
                    matched_truth[:, 0]
                    <= matched_truth[0, 0] + self.alignment_duration
                ) & np.isfinite(estimate[:, 3])
                vertical_offset = 0.0
                if np.any(vertical_mask):
                    vertical_offset = float(
                        np.median(
                            matched_truth[vertical_mask, 3]
                            - estimate[vertical_mask, 3]
                        )
                    )
                    aligned[:, 3] += vertical_offset
                yaw_degrees = math.degrees(angle)
            except (IndexError, ValueError, ZeroDivisionError) as error:
                rospy.logwarn_throttle(
                    5.0, "Evaluation alignment failed for %s: %s", name, error
                )
                continue

            self.publishers[name].publish(
                trajectory_to_path(aligned, self.evaluation_frame)
            )
            if name == "fused":
                self.fused_transform = (
                    angle,
                    float(translation[0]),
                    float(translation[1]),
                    vertical_offset,
                )
            current = (round(yaw_degrees, 6), round(vertical_offset, 6))
            if self.last_alignment.get(name) != current:
                rospy.loginfo(
                    "RViz evaluation alignment %s: yaw=%.3f deg, "
                    "z_offset=%.3f m",
                    name,
                    yaw_degrees,
                    vertical_offset,
                )
                self.last_alignment[name] = current
        self.publish_transform()

    def publish_transform(self):
        if self.fused_transform is None:
            return
        angle, translation_x, translation_y, translation_z = (
            self.fused_transform
        )
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.evaluation_frame
        transform.child_frame_id = self.world_frame
        transform.transform.translation.x = translation_x
        transform.transform.translation.y = translation_y
        transform.transform.translation.z = translation_z
        transform.transform.rotation.z = math.sin(0.5 * angle)
        transform.transform.rotation.w = math.cos(0.5 * angle)
        self.transform_broadcaster.sendTransform(transform)


def main():
    rospy.init_node("evaluation_aligned_path_replay")
    EvaluationAlignedPathReplay()
    rospy.spin()


if __name__ == "__main__":
    main()
