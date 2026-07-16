#!/usr/bin/env python3

import csv
import importlib.util
import math
import threading
import time
from pathlib import Path

import matplotlib
import numpy as np
import rosbag
import rospy
from nav_msgs.msg import Path as RosPath
from std_srvs.srv import Trigger, TriggerResponse


matplotlib.use("Agg")

evaluation_script = Path(__file__).with_name("trajectory_metrics.py")
evaluation_spec = importlib.util.spec_from_file_location(
    "faster_lio_gnss_pose3_evaluation", evaluation_script
)
evaluation = importlib.util.module_from_spec(evaluation_spec)
evaluation_spec.loader.exec_module(evaluation)


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


def write_trajectory(path, trajectory):
    with path.open("w", encoding="ascii") as stream:
        stream.write("# timestamp x y z\n")
        for row in trajectory:
            stream.write(
                f"{row[0]:.9f} {row[1]:.9f} "
                f"{row[2]:.9f} {row[3]:.9f}\n"
            )


def write_metrics(path, reports):
    fieldnames = list(reports[0].keys())
    with path.open("w", encoding="ascii", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(reports)


def improvement_percent(baseline, candidate):
    if not math.isfinite(baseline) or abs(baseline) < 1e-12:
        return float("nan")
    return 100.0 * (baseline - candidate) / baseline


class OnlinePose3ResultEvaluator:
    def __init__(self):
        self.alignment_duration = float(
            rospy.get_param("~alignment_duration", 60.0)
        )
        self.output_directory = Path(
            rospy.get_param(
                "~output_directory",
                str(Path.cwd() / "pose3_online_results"),
            )
        )
        self.bag_file = rospy.get_param("~bag_file", "")
        self.configured_bag_end_time = float(
            rospy.get_param("~bag_end_time", 0.0)
        )
        self.end_tolerance = float(rospy.get_param("~end_tolerance", 0.75))
        self.settle_seconds = float(
            rospy.get_param("~settle_seconds", 3.0)
        )
        self.minimum_samples = int(rospy.get_param("~minimum_samples", 20))
        self.maximum_end_gap = float(
            rospy.get_param("~maximum_end_gap", 2.0)
        )
        self.maximum_wait_after_bag = float(
            rospy.get_param("~maximum_wait_after_bag", 60.0)
        )

        self.messages = {}
        self.lock = threading.Lock()
        self.last_path_update = time.monotonic()
        self.last_ros_time = 0.0
        self.evaluating = False
        self.evaluated = False
        self.bag_end_wall_time = None
        self.bag_end_time = self.load_bag_end_time()
        self.lio_name = rospy.get_param("~lio_name", "Faster-LIO")
        self.gnss_name = rospy.get_param("~gnss_name", "F9P")
        self.fused_name = rospy.get_param(
            "~fused_name", "Pose3-Online"
        )
        self.truth_name = rospy.get_param(
            "~truth_name", "Ground-Truth"
        )

        topics = {
            self.lio_name: rospy.get_param(
                "~lio_path_topic", "/pose3_online/path/lio"
            ),
            self.gnss_name: rospy.get_param(
                "~gnss_path_topic", "/pose3_online/path/gnss"
            ),
            self.fused_name: rospy.get_param(
                "~fused_path_topic", "/pose3_online/path/fused"
            ),
            self.truth_name: rospy.get_param(
                "~truth_path_topic",
                "/pose3_online/path/ground_truth",
            ),
        }
        for name, topic in topics.items():
            rospy.Subscriber(
                topic,
                RosPath,
                self.on_path,
                callback_args=name,
                queue_size=1,
            )

        self.service = rospy.Service(
            "~evaluate", Trigger, self.on_evaluate_service
        )
        self.monitor = threading.Thread(
            target=self.monitor_end_of_run, daemon=True
        )
        self.monitor.start()

        if self.bag_end_time is None:
            rospy.logwarn(
                "Pose3 evaluator has no readable bag end time; "
                "call ~evaluate manually when the run is complete"
            )
        else:
            rospy.loginfo(
                "Pose3 evaluator will run after bag end %.3f",
                self.bag_end_time,
            )

    def load_bag_end_time(self):
        if self.configured_bag_end_time > 0.0:
            return self.configured_bag_end_time
        if not self.bag_file:
            return None
        path = Path(self.bag_file)
        if not path.is_file():
            rospy.logwarn("Evaluation bag does not exist: %s", path)
            return None
        try:
            with rosbag.Bag(str(path), "r") as bag:
                return bag.get_end_time()
        except Exception as error:
            rospy.logerr("Failed to read bag end time: %s", error)
            return None

    def on_path(self, message, name):
        with self.lock:
            self.messages[name] = message
            self.last_path_update = time.monotonic()

    def on_evaluate_service(self, _request):
        success, message = self.evaluate_once("manual service")
        return TriggerResponse(success=success, message=message)

    def monitor_end_of_run(self):
        while not rospy.is_shutdown():
            time.sleep(0.5)
            now = rospy.Time.now().to_sec()
            with self.lock:
                if now + 0.1 < self.last_ros_time:
                    self.evaluated = False
                    self.evaluating = False
                    self.bag_end_wall_time = None
                    self.messages.clear()
                self.last_ros_time = now
                end_reached = (
                    self.bag_end_time is not None
                    and now >= self.bag_end_time - self.end_tolerance
                )
                if end_reached and self.bag_end_wall_time is None:
                    self.bag_end_wall_time = time.monotonic()
                settled = (
                    time.monotonic() - self.last_path_update
                    >= self.settle_seconds
                )
                lio_message = self.messages.get(self.lio_name)
                lio_end_time = (
                    lio_message.poses[-1].header.stamp.to_sec()
                    if lio_message is not None and lio_message.poses
                    else 0.0
                )
                coverage_ready = (
                    self.bag_end_time is not None
                    and lio_end_time
                    >= self.bag_end_time - self.maximum_end_gap
                )
                timed_out = (
                    self.bag_end_wall_time is not None
                    and time.monotonic() - self.bag_end_wall_time
                    >= self.maximum_wait_after_bag
                )
                should_evaluate = (
                    end_reached
                    and settled
                    and (coverage_ready or timed_out)
                    and not self.evaluated
                    and not self.evaluating
                )
            if should_evaluate:
                self.evaluate_once("automatic bag completion")

    def snapshot_trajectories(self):
        with self.lock:
            messages = dict(self.messages)
        required = (
            self.lio_name,
            self.gnss_name,
            self.fused_name,
            self.truth_name,
        )
        missing = [name for name in required if name not in messages]
        if missing:
            raise RuntimeError("missing path topics: " + ", ".join(missing))

        trajectories = {
            name: path_to_trajectory(messages[name]) for name in required
        }
        short = [
            f"{name}={len(trajectory)}"
            for name, trajectory in trajectories.items()
            if len(trajectory) < self.minimum_samples
        ]
        if short:
            raise RuntimeError(
                "not enough trajectory samples: " + ", ".join(short)
            )
        return trajectories

    def evaluate_once(self, reason):
        with self.lock:
            if self.evaluating:
                return False, "evaluation is already running"
            self.evaluating = True

        try:
            trajectories = self.snapshot_trajectories()
            truth = trajectories[self.truth_name]
            evaluated = []
            for name in (
                self.lio_name,
                self.gnss_name,
                self.fused_name,
            ):
                metrics, estimate, matched_truth = evaluation.evaluate(
                    name,
                    trajectories[name],
                    truth,
                    self.alignment_duration,
                )
                evaluated.append((metrics, estimate, name, matched_truth))

            self.output_directory.mkdir(parents=True, exist_ok=True)
            filenames = {
                self.lio_name: "faster_lio.txt",
                self.gnss_name: "gnss.txt",
                self.fused_name: "pose3_isam2.txt",
                self.truth_name: "ground_truth.txt",
            }
            for name, trajectory in trajectories.items():
                write_trajectory(
                    self.output_directory / filenames[name], trajectory
                )
            reports = [item[0] for item in evaluated]
            write_metrics(self.output_directory / "metrics.csv", reports)
            evaluation.save_plot(
                self.output_directory / "trajectory_comparison.png",
                evaluated,
            )

            report_by_name = {report["name"]: report for report in reports}
            baseline = report_by_name[self.lio_name]
            fused = report_by_name[self.fused_name]
            horizontal_improvement = improvement_percent(
                baseline["horizontal_rmse_m"],
                fused["horizontal_rmse_m"],
            )
            vertical_improvement = improvement_percent(
                baseline["vertical_rmse_m"],
                fused["vertical_rmse_m"],
            )
            three_dimensional_improvement = improvement_percent(
                baseline["ate_3d_rmse_m"],
                fused["ate_3d_rmse_m"],
            )
            rpe_improvement = improvement_percent(
                baseline["rpe_100m"],
                fused["rpe_100m"],
            )

            summary = (
                "Online causal Pose3 evaluation\n"
                f"trigger: {reason}\n"
                "protocol: one fixed initial-window SE(2) alignment plus "
                "one vertical offset, "
                f"duration={self.alignment_duration:.1f}s\n\n"
                f"{self.lio_name} horizontal RMSE: "
                f"{baseline['horizontal_rmse_m']:.3f} m\n"
                f"{self.fused_name} horizontal RMSE: "
                f"{fused['horizontal_rmse_m']:.3f} m\n"
                f"horizontal improvement: {horizontal_improvement:.2f}%\n\n"
                f"{self.lio_name} vertical RMSE: "
                f"{baseline['vertical_rmse_m']:.3f} m\n"
                f"{self.fused_name} vertical RMSE: "
                f"{fused['vertical_rmse_m']:.3f} m\n"
                f"vertical improvement: {vertical_improvement:.2f}%\n\n"
                f"{self.lio_name} 3D RMSE: "
                f"{baseline['ate_3d_rmse_m']:.3f} m\n"
                f"{self.fused_name} 3D RMSE: "
                f"{fused['ate_3d_rmse_m']:.3f} m\n"
                f"3D improvement: {three_dimensional_improvement:.2f}%\n\n"
                f"{self.lio_name} RPE 100 m: "
                f"{baseline['rpe_100m']:.3f} m\n"
                f"{self.fused_name} RPE 100 m: "
                f"{fused['rpe_100m']:.3f} m\n"
                f"RPE 100 m improvement: {rpe_improvement:.2f}%\n"
            )
            with (
                self.output_directory / "comparison.txt"
            ).open("w", encoding="ascii") as stream:
                stream.write(summary)

            for report in reports:
                rospy.loginfo(
                    "%s: horizontal=%.3fm vertical=%.3fm "
                    "3D=%.3fm RPE100=%.3fm",
                    report["name"],
                    report["horizontal_rmse_m"],
                    report["vertical_rmse_m"],
                    report["ate_3d_rmse_m"],
                    report["rpe_100m"],
                )
            rospy.loginfo(
                "%s improvement over %s: "
                "horizontal=%.2f%% vertical=%.2f%% "
                "3D=%.2f%% RPE100=%.2f%%",
                self.fused_name,
                self.lio_name,
                horizontal_improvement,
                vertical_improvement,
                three_dimensional_improvement,
                rpe_improvement,
            )
            rospy.loginfo(
                "Pose3 online evaluation written to %s",
                self.output_directory,
            )
            with self.lock:
                self.evaluated = True
            return True, str(self.output_directory)
        except Exception as error:
            rospy.logerr("Pose3 online trajectory evaluation failed: %s", error)
            return False, str(error)
        finally:
            with self.lock:
                self.evaluating = False


def main():
    rospy.init_node("online_pose3_result_evaluator")
    OnlinePose3ResultEvaluator()
    rospy.spin()


if __name__ == "__main__":
    main()
