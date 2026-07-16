#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import numpy as np


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


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
            (stamp, *geodetic_to_enu(latitude, longitude, height, reference))
            for stamp, latitude, longitude, height in geodetic
        ]
    )


def load_trajectory(path):
    trajectory = []
    with open(path, encoding="ascii") as stream:
        for line in stream:
            fields = line.split()
            if not fields or fields[0].startswith("#"):
                continue
            try:
                stamp, x, y = map(float, fields[:3])
                z = float(fields[3]) if len(fields) >= 8 else float("nan")
            except (ValueError, IndexError):
                continue
            trajectory.append((stamp, x, y, z))
    if len(trajectory) < 2:
        raise RuntimeError(f"not enough trajectory samples in {path}")
    return np.asarray(trajectory)


def interpolate_to_truth(trajectory, ground_truth):
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


def robust_xy_alignment(estimate, truth, duration):
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
            * np.sum(source_centered * target_centered, axis=1)
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


def align_trajectory(estimate, truth, duration):
    angle, translation = robust_xy_alignment(estimate, truth, duration)
    rotation = np.array(
        [
            [math.cos(angle), -math.sin(angle)],
            [math.sin(angle), math.cos(angle)],
        ]
    )
    aligned = estimate.copy()
    aligned[:, 1:3] = estimate[:, 1:3] @ rotation.T + translation
    vertical_mask = (truth[:, 0] <= truth[0, 0] + duration) & np.isfinite(
        estimate[:, 3]
    )
    vertical_offset = float("nan")
    if np.any(vertical_mask):
        vertical_offset = np.median(
            truth[vertical_mask, 3] - estimate[vertical_mask, 3]
        )
        aligned[:, 3] += vertical_offset
    return aligned, math.degrees(angle), vertical_offset


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


def course_rmse(estimate, truth):
    truth_delta = truth[2:, 1:3] - truth[:-2, 1:3]
    estimate_delta = estimate[2:, 1:3] - estimate[:-2, 1:3]
    valid = np.linalg.norm(truth_delta, axis=1) > 2.0
    truth_course = np.arctan2(truth_delta[valid, 1], truth_delta[valid, 0])
    estimate_course = np.arctan2(
        estimate_delta[valid, 1], estimate_delta[valid, 0]
    )
    difference = (estimate_course - truth_course + np.pi) % (2 * np.pi) - np.pi
    return math.degrees(math.sqrt(np.mean(difference * difference)))


def evaluate(name, trajectory, ground_truth, alignment_duration):
    estimate, truth = interpolate_to_truth(trajectory, ground_truth)
    estimate, yaw, vertical_offset = align_trajectory(
        estimate, truth, alignment_duration
    )
    horizontal_error = np.linalg.norm(estimate[:, 1:3] - truth[:, 1:3], axis=1)
    horizontal_length = np.sum(
        np.linalg.norm(np.diff(estimate[:, 1:3], axis=0), axis=1)
    )
    truth_horizontal_length = np.sum(
        np.linalg.norm(np.diff(truth[:, 1:3], axis=0), axis=1)
    )
    metrics = {
        "name": name,
        "samples": len(truth),
        "duration_s": truth[-1, 0] - truth[0, 0],
        "alignment_yaw_deg": yaw,
        "horizontal_rmse_m": math.sqrt(np.mean(horizontal_error**2)),
        "horizontal_median_m": np.median(horizontal_error),
        "horizontal_p95_m": np.percentile(horizontal_error, 95),
        "horizontal_max_m": np.max(horizontal_error),
        "horizontal_endpoint_m": horizontal_error[-1],
        "horizontal_length_error_percent": 100.0
        * (horizontal_length / truth_horizontal_length - 1.0),
        "course_rmse_deg": course_rmse(estimate, truth),
        "rpe_10m": segment_rpe(estimate, truth, 10.0),
        "rpe_50m": segment_rpe(estimate, truth, 50.0),
        "rpe_100m": segment_rpe(estimate, truth, 100.0),
        "vertical_offset_m": vertical_offset,
        "vertical_rmse_m": float("nan"),
        "vertical_p95_m": float("nan"),
        "vertical_endpoint_m": float("nan"),
        "ate_3d_rmse_m": float("nan"),
    }
    if np.all(np.isfinite(estimate[:, 3])):
        vertical_error = np.abs(estimate[:, 3] - truth[:, 3])
        error_3d = np.linalg.norm(estimate[:, 1:4] - truth[:, 1:4], axis=1)
        metrics.update(
            {
                "vertical_rmse_m": math.sqrt(np.mean(vertical_error**2)),
                "vertical_p95_m": np.percentile(vertical_error, 95),
                "vertical_endpoint_m": vertical_error[-1],
                "ate_3d_rmse_m": math.sqrt(np.mean(error_3d**2)),
            }
        )
    return metrics, estimate, truth


def save_plot(path, evaluated):
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, 2, figsize=(14, 6))
    truth = evaluated[0][3]
    axes[0].plot(truth[:, 1], truth[:, 2], color="black", linewidth=2, label="Truth")
    colors = [
        "#1f77b4",
        "#d62728",
        "#2ca02c",
        "#9467bd",
        "#ff7f0e",
        "#17becf",
        "#8c564b",
    ]
    for index, (metrics, estimate, _, _) in enumerate(evaluated):
        axes[0].plot(
            estimate[:, 1],
            estimate[:, 2],
            linewidth=1.1,
            color=colors[index % len(colors)],
            label=metrics["name"],
        )
        if np.all(np.isfinite(estimate[:, 3])):
            axes[1].plot(
                estimate[:, 0] - estimate[0, 0],
                estimate[:, 3],
                linewidth=1.1,
                color=colors[index % len(colors)],
                label=metrics["name"],
            )
    axes[1].plot(
        truth[:, 0] - truth[0, 0],
        truth[:, 3],
        color="black",
        linewidth=2,
        label="Truth",
    )
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].set_xlabel("East [m]")
    axes[0].set_ylabel("North [m]")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Up [m]")
    for axis in axes:
        axis.grid(True, linewidth=0.4, alpha=0.5)
        axis.legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate trajectories with fixed initial-window alignment."
    )
    parser.add_argument("--ground_truth", required=True)
    parser.add_argument("--trajectory", action="append", default=[], required=True)
    parser.add_argument("--alignment_duration", type=float, default=60.0)
    parser.add_argument("--report", required=True)
    parser.add_argument("--plot")
    args = parser.parse_args()

    ground_truth = load_ground_truth(args.ground_truth)
    evaluated = []
    for specification in args.trajectory:
        if "=" not in specification:
            raise RuntimeError("--trajectory must use NAME=PATH")
        name, path = specification.split("=", 1)
        metrics, estimate, truth = evaluate(
            name,
            load_trajectory(path),
            ground_truth,
            args.alignment_duration,
        )
        evaluated.append((metrics, estimate, name, truth))
        print(
            f"{name}: horizontal={metrics['horizontal_rmse_m']:.3f} m, "
            f"vertical={metrics['vertical_rmse_m']:.3f} m, "
            f"3D={metrics['ate_3d_rmse_m']:.3f} m, "
            f"RPE100={metrics['rpe_100m']:.3f} m"
        )

    output = Path(args.report)
    output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(evaluated[0][0].keys())
    with output.open("w", encoding="ascii", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for metrics, _, _, _ in evaluated:
            writer.writerow(metrics)

    if args.plot:
        plot_path = Path(args.plot)
        plot_path.parent.mkdir(parents=True, exist_ok=True)
        save_plot(plot_path, evaluated)


if __name__ == "__main__":
    main()
