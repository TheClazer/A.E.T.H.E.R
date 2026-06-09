#!/usr/bin/env python3
# KITTI-style translational drift (the DP7 < 1.5% / 200 m metric) + ATE, from two
# TUM-format trajectories (timestamp tx ty tz qx qy qz qw).
#
#   python eval/compute_drift.py groundtruth.txt estimate.txt
#
# Computes segment-averaged % drift over {10,20,40,80} m windows and terminal ATE.
# (Cross-check against `evo_rpe ... --delta L --delta_unit m --all_pairs` and
#  `rosrun ov_eval error_singlerun`.)
import sys
import numpy as np


def load_tum(path):
    d = np.loadtxt(path)
    return d[:, 0], d[:, 1:4]            # t, xyz


def cumulative_distance(xyz):
    seg = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def segment_drift(gt, est, seg_len):
    dist = cumulative_distance(gt)
    errs = []
    j = 0
    for i in range(len(gt)):
        target = dist[i] + seg_len
        while j < len(gt) and dist[j] < target:
            j += 1
        if j >= len(gt):
            break
        d_est = np.linalg.norm((est[j] - est[i]) - (gt[j] - gt[i]))
        errs.append(d_est / seg_len)
    return float(np.mean(errs) * 100.0) if errs else float('nan')


def main():
    if len(sys.argv) != 3:
        print("usage: compute_drift.py groundtruth.txt estimate.txt"); sys.exit(1)
    _, gt = load_tum(sys.argv[1])
    _, est = load_tum(sys.argv[2])
    n = min(len(gt), len(est)); gt, est = gt[:n], est[:n]
    est = est - est[0] + gt[0]          # origin-align (honest accumulated drift)

    print("A.E.T.H.E.R drift report")
    print("-" * 40)
    total = cumulative_distance(gt)[-1]
    print(f"  path length            {total:8.1f} m")
    for L in (10, 20, 40, 80):
        print(f"  drift over {L:3d} m seg   {segment_drift(gt, est, L):8.3f} %")
    ate = float(np.sqrt(np.mean(np.sum((est - gt) ** 2, axis=1))))
    terminal = float(np.linalg.norm(est[-1] - gt[-1]))
    print(f"  RMS ATE                {ate:8.3f} m")
    print(f"  terminal error         {terminal:8.3f} m   (DP7 gate <= 3.0 m)")
    print("-" * 40)
    print("  PASS" if terminal <= 3.0 else "  REVIEW: terminal > 3.0 m")


if __name__ == "__main__":
    main()
