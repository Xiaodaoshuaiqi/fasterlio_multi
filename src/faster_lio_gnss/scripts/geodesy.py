#!/usr/bin/env python3

import argparse
import datetime
import math
from pathlib import Path

import numpy as np


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def normalize_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def geodetic_to_ecef(latitude, longitude, height):
    latitude = math.radians(latitude)
    longitude = math.radians(longitude)
    sin_latitude = math.sin(latitude)
    prime_vertical = WGS84_A / math.sqrt(
        1.0 - WGS84_E2 * sin_latitude * sin_latitude
    )
    return np.array(
        [
            (prime_vertical + height)
            * math.cos(latitude)
            * math.cos(longitude),
            (prime_vertical + height)
            * math.cos(latitude)
            * math.sin(longitude),
            (prime_vertical * (1.0 - WGS84_E2) + height)
            * math.sin(latitude),
        ]
    )


def geodetic_to_enu(latitude, longitude, height, reference):
    ref_latitude, ref_longitude, ref_height = reference
    delta = geodetic_to_ecef(latitude, longitude, height) - geodetic_to_ecef(
        ref_latitude, ref_longitude, ref_height
    )
    latitude_rad = math.radians(ref_latitude)
    longitude_rad = math.radians(ref_longitude)
    rotation = np.array(
        [
            [-math.sin(longitude_rad), math.cos(longitude_rad), 0.0],
            [
                -math.sin(latitude_rad) * math.cos(longitude_rad),
                -math.sin(latitude_rad) * math.sin(longitude_rad),
                math.cos(latitude_rad),
            ],
            [
                math.cos(latitude_rad) * math.cos(longitude_rad),
                math.cos(latitude_rad) * math.sin(longitude_rad),
                math.sin(latitude_rad),
            ],
        ]
    )
    return rotation @ delta


def load_ground_truth(path):
    geodetic = []
    with open(path, encoding="ascii") as stream:
        for line in stream:
            fields = line.split()
            try:
                stamp = float(fields[0])
                latitude = (
                    float(fields[3])
                    + float(fields[4]) / 60.0
                    + float(fields[5]) / 3600.0
                )
                longitude = (
                    float(fields[6])
                    + float(fields[7]) / 60.0
                    + float(fields[8]) / 3600.0
                )
                height = float(fields[9])
            except (ValueError, IndexError):
                continue
            geodetic.append((stamp, latitude, longitude, height))
    if not geodetic:
        raise RuntimeError(f"no ground-truth samples in {path}")
    reference = geodetic[0][1:4]
    return np.array(
        [
            (stamp, *geodetic_to_enu(latitude, longitude, height, reference)[:2])
            for stamp, latitude, longitude, height in geodetic
        ]
    )


def load_lio(path):
    trajectory = []
    with open(path, encoding="ascii") as stream:
        for line in stream:
            fields = line.split()
            if not fields or fields[0].startswith("#"):
                continue
            try:
                stamp, x, y, _, qx, qy, qz, qw = map(float, fields[:8])
            except (ValueError, IndexError):
                continue
            yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz),
            )
            trajectory.append((stamp, x, y, yaw))
    result = np.array(trajectory)
    result[:, 3] = np.unwrap(result[:, 3])
    return result


def load_xy_yaw(path):
    trajectory = []
    with open(path, encoding="ascii") as stream:
        for line in stream:
            fields = line.split()
            if not fields or fields[0].startswith("#"):
                continue
            try:
                trajectory.append(tuple(map(float, fields[:4])))
            except (ValueError, IndexError):
                continue
    result = np.array(trajectory)
    result[:, 3] = np.unwrap(result[:, 3])
    return result


def nmea_checksum_valid(line):
    dollar = line.find("$")
    star = line.find("*", dollar)
    if dollar < 0 or star < 0 or star + 2 >= len(line):
        return False
    checksum = 0
    for character in line[dollar + 1 : star]:
        checksum ^= ord(character)
    try:
        expected = int(line[star + 1 : star + 3], 16)
    except ValueError:
        return False
    return checksum == expected


def nmea_degrees(value, hemisphere):
    degrees_minutes = float(value)
    integer_degrees = int(degrees_minutes / 100.0)
    result = integer_degrees + (degrees_minutes - integer_degrees * 100.0) / 60.0
    return -result if hemisphere in ("S", "W") else result


def load_nmea(path, date_utc):
    date = datetime.datetime.strptime(date_utc, "%Y-%m-%d").replace(
        tzinfo=datetime.timezone.utc
    )
    samples = []
    previous_seconds = -1.0
    day_offset = 0
    with open(path, encoding="ascii", errors="ignore") as stream:
        for line in stream:
            if not nmea_checksum_valid(line):
                continue
            body = line[line.find("$") + 1 : line.find("*")]
            fields = body.split(",")
            if not fields[0].endswith("GGA") or len(fields) < 12:
                continue
            try:
                hhmmss = float(fields[1])
                hour = int(hhmmss / 10000.0)
                minute = int((hhmmss - hour * 10000.0) / 100.0)
                second = hhmmss - hour * 10000.0 - minute * 100.0
                seconds = hour * 3600.0 + minute * 60.0 + second
                latitude = nmea_degrees(fields[2], fields[3])
                longitude = nmea_degrees(fields[4], fields[5])
                quality = int(fields[6])
                height = float(fields[9]) + float(fields[11])
            except (ValueError, IndexError):
                continue
            if quality != 2:
                continue
            if previous_seconds >= 0.0 and seconds + 12.0 * 3600.0 < previous_seconds:
                day_offset += 1
            previous_seconds = seconds
            stamp = date.timestamp() + day_offset * 86400.0 + seconds
            samples.append((stamp, latitude, longitude, height))
    if not samples:
        raise RuntimeError(f"no accepted GGA samples in {path}")
    reference = samples[0][1:4]
    positions = np.array(
        [
            (stamp, *geodetic_to_enu(latitude, longitude, height, reference)[:2])
            for stamp, latitude, longitude, height in samples
        ]
    )
    deltas = np.diff(positions[:, 1:3], axis=0)
    course = np.zeros(len(positions))
    course[1:] = np.arctan2(deltas[:, 1], deltas[:, 0])
    course[0] = course[1]
    course = np.unwrap(course)
    return np.column_stack((positions, course))


def interpolate_to_ground_truth(trajectory, ground_truth):
    mask = (ground_truth[:, 0] >= trajectory[0, 0]) & (
        ground_truth[:, 0] <= trajectory[-1, 0]
    )
    truth = ground_truth[mask]
    estimate = np.column_stack(
        [
            truth[:, 0],
            np.interp(truth[:, 0], trajectory[:, 0], trajectory[:, 1]),
            np.interp(truth[:, 0], trajectory[:, 0], trajectory[:, 2]),
            np.interp(truth[:, 0], trajectory[:, 0], trajectory[:, 3]),
        ]
    )
    return estimate, truth


def robust_initial_alignment(estimate, truth, duration):
    mask = truth[:, 0] <= truth[0, 0] + duration
    source = estimate[mask, 1:3]
    target = truth[mask, 1:3]
    weights = np.ones(len(source))
    angle = 0.0
    translation = np.zeros(2)
    for _ in range(10):
        weight_sum = np.sum(weights)
        source_mean = np.sum(source * weights[:, None], axis=0) / weight_sum
        target_mean = np.sum(target * weights[:, None], axis=0) / weight_sum
        source_centered = source - source_mean
        target_centered = target - target_mean
        cross = np.sum(
            weights
            * (
                source_centered[:, 0] * target_centered[:, 1]
                - source_centered[:, 1] * target_centered[:, 0]
            )
        )
        dot = np.sum(
            weights
            * (
                source_centered[:, 0] * target_centered[:, 0]
                + source_centered[:, 1] * target_centered[:, 1]
            )
        )
        angle = math.atan2(cross, dot)
        rotation = np.array(
            [
                [math.cos(angle), -math.sin(angle)],
                [math.sin(angle), math.cos(angle)],
            ]
        )
        translation = target_mean - rotation @ source_mean
        residual = np.linalg.norm(source @ rotation.T + translation - target, axis=1)
        weights = np.minimum(1.0, 2.0 / np.maximum(residual, 1e-9))
    return angle, translation


def aligned_trajectory(estimate, truth, alignment_duration):
    angle, translation = robust_initial_alignment(
        estimate, truth, alignment_duration
    )
    rotation = np.array(
        [
            [math.cos(angle), -math.sin(angle)],
            [math.sin(angle), math.cos(angle)],
        ]
    )
    aligned = estimate.copy()
    aligned[:, 1:3] = estimate[:, 1:3] @ rotation.T + translation
    aligned[:, 3] = estimate[:, 3] + angle
    return aligned, math.degrees(angle)


def segment_rpe(estimate, truth, distance):
    truth_steps = np.linalg.norm(np.diff(truth[:, 1:3], axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(truth_steps)))
    errors = []
    for start in range(len(truth)):
        end = np.searchsorted(cumulative, cumulative[start] + distance)
        if end >= len(truth):
            break
        truth_delta = truth[end, 1:3] - truth[start, 1:3]
        estimate_delta = estimate[end, 1:3] - estimate[start, 1:3]
        errors.append(np.linalg.norm(estimate_delta - truth_delta))
    if not errors:
        return float("nan")
    errors = np.asarray(errors)
    return math.sqrt(np.mean(errors * errors))


def course_error(estimate, truth):
    truth_delta = truth[2:, 1:3] - truth[:-2, 1:3]
    estimate_delta = estimate[2:, 1:3] - estimate[:-2, 1:3]
    valid = np.linalg.norm(truth_delta, axis=1) > 2.0
    truth_course = np.arctan2(truth_delta[valid, 1], truth_delta[valid, 0])
    estimate_course = np.arctan2(
        estimate_delta[valid, 1], estimate_delta[valid, 0]
    )
    difference = normalize_angle(estimate_course - truth_course)
    return math.degrees(math.sqrt(np.mean(difference * difference)))


def curvature_error(estimate, truth):
    def signed_curvature(points):
        first = points[1:-1] - points[:-2]
        second = points[2:] - points[1:-1]
        chord = points[2:] - points[:-2]
        denominator = (
            np.linalg.norm(first, axis=1)
            * np.linalg.norm(second, axis=1)
            * np.linalg.norm(chord, axis=1)
        )
        cross = first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0]
        curvature = np.zeros(len(cross))
        valid = denominator > 4.0
        curvature[valid] = 2.0 * cross[valid] / denominator[valid]
        return curvature, valid

    truth_curvature, valid = signed_curvature(truth[:, 1:3])
    estimate_curvature, _ = signed_curvature(estimate[:, 1:3])
    difference = estimate_curvature[valid] - truth_curvature[valid]
    return math.sqrt(np.mean(difference * difference))


def evaluate(name, trajectory, ground_truth, alignment_duration):
    estimate, truth = interpolate_to_ground_truth(trajectory, ground_truth)
    estimate, alignment_angle = aligned_trajectory(
        estimate, truth, alignment_duration
    )
    errors = np.linalg.norm(estimate[:, 1:3] - truth[:, 1:3], axis=1)
    estimate_length = np.sum(np.linalg.norm(np.diff(estimate[:, 1:3], axis=0), axis=1))
    truth_length = np.sum(np.linalg.norm(np.diff(truth[:, 1:3], axis=0), axis=1))
    return {
        "name": name,
        "samples": len(errors),
        "duration": truth[-1, 0] - truth[0, 0],
        "alignment_yaw": alignment_angle,
        "rmse": math.sqrt(np.mean(errors * errors)),
        "median": np.median(errors),
        "p95": np.percentile(errors, 95),
        "maximum": np.max(errors),
        "endpoint": errors[-1],
        "estimate_length": estimate_length,
        "truth_length": truth_length,
        "length_error_percent": 100.0 * (estimate_length / truth_length - 1.0),
        "course_rmse": course_error(estimate, truth),
        "curvature_rmse": curvature_error(estimate, truth),
        "rpe_10": segment_rpe(estimate, truth, 10.0),
        "rpe_50": segment_rpe(estimate, truth, 50.0),
        "rpe_100": segment_rpe(estimate, truth, 100.0),
    }


def print_report(metrics):
    print(f"\n[{metrics['name']}]")
    print(
        f"samples={metrics['samples']} duration={metrics['duration']:.1f}s "
        f"initial_alignment_yaw={metrics['alignment_yaw']:.3f}deg"
    )
    print(
        f"ATE horizontal: RMSE={metrics['rmse']:.3f}m "
        f"median={metrics['median']:.3f}m p95={metrics['p95']:.3f}m "
        f"max={metrics['maximum']:.3f}m endpoint={metrics['endpoint']:.3f}m"
    )
    print(
        f"path length: estimate={metrics['estimate_length']:.3f}m "
        f"truth={metrics['truth_length']:.3f}m "
        f"error={metrics['length_error_percent']:.3f}%"
    )
    print(
        f"shape: course_RMSE={metrics['course_rmse']:.3f}deg "
        f"curvature_RMSE={metrics['curvature_rmse']:.5f}1/m "
        f"RPE10={metrics['rpe_10']:.3f}m "
        f"RPE50={metrics['rpe_50']:.3f}m "
        f"RPE100={metrics['rpe_100']:.3f}m"
    )


def save_plot(path, trajectories, ground_truth, alignment_duration):
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(10, 8))
    axis.plot(
        ground_truth[:, 1],
        ground_truth[:, 2],
        color="black",
        linewidth=2.0,
        label="SPAN-CPT truth",
    )
    colors = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd"]
    for index, (name, trajectory) in enumerate(trajectories):
        estimate, truth = interpolate_to_ground_truth(trajectory, ground_truth)
        estimate, _ = aligned_trajectory(estimate, truth, alignment_duration)
        axis.plot(
            estimate[:, 1],
            estimate[:, 2],
            color=colors[index % len(colors)],
            linewidth=1.2,
            label=name,
        )
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("East [m]")
    axis.set_ylabel("North [m]")
    axis.grid(True, linewidth=0.4, alpha=0.5)
    axis.legend()
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate trajectories after one fixed initial-window SE(2) alignment."
    )
    parser.add_argument("--ground_truth", required=True)
    parser.add_argument("--lio")
    parser.add_argument("--nmea")
    parser.add_argument("--nmea_date", default="2021-05-17")
    parser.add_argument("--fused", action="append", default=[])
    parser.add_argument("--alignment_duration", type=float, default=60.0)
    parser.add_argument("--report")
    parser.add_argument("--plot")
    args = parser.parse_args()

    ground_truth = load_ground_truth(args.ground_truth)
    trajectories = []
    if args.lio:
        trajectories.append(("Faster-LIO", load_lio(args.lio)))
    if args.nmea:
        trajectories.append(("F9P-NMEA", load_nmea(args.nmea, args.nmea_date)))
    for specification in args.fused:
        if "=" not in specification:
            raise RuntimeError("--fused must use NAME=PATH")
        name, path = specification.split("=", 1)
        trajectories.append((name, load_xy_yaw(path)))

    reports = [
        evaluate(name, trajectory, ground_truth, args.alignment_duration)
        for name, trajectory in trajectories
    ]
    for report in reports:
        print_report(report)

    if args.plot:
        output = Path(args.plot)
        output.parent.mkdir(parents=True, exist_ok=True)
        save_plot(output, trajectories, ground_truth, args.alignment_duration)

    if args.report:
        output = Path(args.report)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", encoding="ascii") as stream:
            header = [
                "name",
                "samples",
                "duration_s",
                "ate_rmse_m",
                "median_m",
                "p95_m",
                "max_m",
                "endpoint_m",
                "length_m",
                "truth_length_m",
                "length_error_percent",
                "course_rmse_deg",
                "curvature_rmse_1pm",
                "rpe_10m",
                "rpe_50m",
                "rpe_100m",
            ]
            stream.write(",".join(header) + "\n")
            for item in reports:
                values = [
                    item["name"],
                    item["samples"],
                    item["duration"],
                    item["rmse"],
                    item["median"],
                    item["p95"],
                    item["maximum"],
                    item["endpoint"],
                    item["estimate_length"],
                    item["truth_length"],
                    item["length_error_percent"],
                    item["course_rmse"],
                    item["curvature_rmse"],
                    item["rpe_10"],
                    item["rpe_50"],
                    item["rpe_100"],
                ]
                stream.write(",".join(map(str, values)) + "\n")


if __name__ == "__main__":
    main()
