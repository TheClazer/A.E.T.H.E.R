#!/usr/bin/env python3
# KITTI-style translational drift (the DP7 < 1.5% / 200 m metric) + ATE, from two
# TUM-format trajectories (timestamp tx ty tz qx qy qz qw).
#
#   python eval/compute_drift.py groundtruth.txt estimate.txt
#
# Pipeline: time-interpolate GT onto the estimate's timestamps, rigidly align
# (Umeyama SE(3), rotation+translation, NO scale) — necessary because a VIO's
# world frame has arbitrary initial yaw — then report segment-averaged % drift
# over {10,20,40,80} m windows, RMS ATE and terminal error. A single rigid fit
# cannot hide accumulating drift, so the numbers stay honest.
# (Cross-check against `evo_ape -a` / `evo_rpe` and `ov_eval error_singlerun`.)
import sys
import numpy as np


def load_tum(path):
    d = np.loadtxt(path)
    if d.ndim != 2 or d.shape[0] < 10:
        print(f"ERROR: {path} has too few poses ({0 if d.ndim != 2 else d.shape[0]})")
        sys.exit(2)
    return d[:, 0], d[:, 1:4]            # t, xyz


def interp_to(t_ref, t_src, xyz_src):
    """Linearly interpolate xyz_src(t_src) onto t_ref (only inside the overlap)."""
    lo, hi = max(t_ref[0], t_src[0]), min(t_ref[-1], t_src[-1])
    m = (t_ref >= lo) & (t_ref <= hi)
    out = np.stack([np.interp(t_ref[m], t_src, xyz_src[:, k]) for k in range(3)], axis=1)
    return t_ref[m], out, m


def umeyama_rigid(src, dst):
    """R, t minimizing ||R@src + t - dst|| (no scale)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    S = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ S @ U.T
    return R, mu_d - R @ mu_s


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
    t_g, gt_raw = load_tum(sys.argv[1])
    t_e, est = load_tum(sys.argv[2])
    # GT interpolated onto the estimate's clock (rates differ: GT ~44 Hz, VIO ~20 Hz)
    t, gt, m = interp_to(t_e, t_g, gt_raw)
    est = est[m]
    # rigid alignment (rotation + translation, no scale): VIO yaw is arbitrary
    R, tr = umeyama_rigid(est, gt)
    est = (R @ est.T).T + tr

    print("A.E.T.H.E.R drift report  (time-synced, SE(3)-aligned, no scale)")
    print("-" * 52)
    total = cumulative_distance(gt)[-1]
    print(f"  matched poses          {len(gt):8d}")
    print(f"  path length            {total:8.1f} m")
    for L in (10, 20, 40, 80):
        print(f"  drift over {L:3d} m seg   {segment_drift(gt, est, L):8.3f} %")
    ate = float(np.sqrt(np.mean(np.sum((est - gt) ** 2, axis=1))))
    terminal = float(np.linalg.norm(est[-1] - gt[-1]))
    drift_pct_total = terminal / total * 100.0 if total > 0 else float('nan')
    print(f"  RMS ATE                {ate:8.3f} m")
    print(f"  terminal error         {terminal:8.3f} m   over {total:.0f} m "
          f"= {drift_pct_total:.3f} %   (DP7: < 1.5 % / 200 m)")
    print("-" * 52)
    print("  PASS" if (terminal <= 3.0 and drift_pct_total <= 1.5) else "  REVIEW: above the DP7 gate")


if __name__ == "__main__":
    main()
