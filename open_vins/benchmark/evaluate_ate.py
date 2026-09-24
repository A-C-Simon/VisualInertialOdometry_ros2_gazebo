#!/usr/bin/env python3
"""Rigidly align a TUM trajectory to EuRoC ground truth and report position ATE."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def load_estimate(path: Path):
    rows = np.loadtxt(path, comments="#")
    return np.atleast_2d(rows)[:, :4]


def load_ground_truth(path: Path):
    rows = []
    with path.open(newline="") as stream:
        for row in csv.reader(line for line in stream if not line.startswith("#")):
            if row:
                rows.append([int(row[0]) * 1e-9, float(row[1]), float(row[2]), float(row[3])])
    return np.asarray(rows)


def associate(estimate, truth, max_delta):
    truth_times = truth[:, 0]
    est_positions = []
    truth_positions = []
    for row in estimate:
        index = int(np.searchsorted(truth_times, row[0]))
        candidates = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(truth)]
        nearest = min(candidates, key=lambda candidate: abs(truth_times[candidate] - row[0]))
        if abs(truth_times[nearest] - row[0]) <= max_delta:
            est_positions.append(row[1:4])
            truth_positions.append(truth[nearest, 1:4])
    return np.asarray(est_positions), np.asarray(truth_positions)


def rigid_align(source, target):
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    return (rotation @ source.T).T + translation


def path_length(points):
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("estimate", type=Path)
    parser.add_argument("ground_truth", type=Path)
    parser.add_argument("--max-time-delta", type=float, default=0.01)
    args = parser.parse_args()

    estimate, truth = associate(
        load_estimate(args.estimate), load_ground_truth(args.ground_truth), args.max_time_delta
    )
    if len(estimate) < 3:
        raise SystemExit(f"only {len(estimate)} timestamp matches")
    aligned = rigid_align(estimate, truth)
    errors = np.linalg.norm(aligned - truth, axis=1)
    result = {
        "matched_poses": len(errors),
        "ate_rmse_m": round(float(np.sqrt(np.mean(errors**2))), 6),
        "ate_median_m": round(float(np.median(errors)), 6),
        "ate_max_m": round(float(np.max(errors)), 6),
        "estimated_path_m": round(path_length(aligned), 3),
        "ground_truth_path_m": round(path_length(truth), 3),
        "alignment": "SE(3), no scale",
    }
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
