#!/usr/bin/env python3
# Generate the A.E.T.H.E.R performance-report figures.
#
# Two modes:
#   (a) offline (no ROS2) — reproduce the synthetic integrity figures:
#         python eval/make_report_figures.py --offline
#       (delegates to offline_demo/run_demo.py)
#   (b) from data — trajectory + drift from TUM files:
#         python eval/make_report_figures.py groundtruth.txt estimate.txt
#
# The 8-Jun deck figures are ILLUSTRATIVE/TARGET; the measured ones come from the
# Ubuntu build (EuRoC + Gazebo) — see docs/AETHER_MANUAL_STEPS.pdf.
import os
import sys


def offline():
    here = os.path.dirname(__file__)
    sys.path.insert(0, os.path.join(here, '..', 'offline_demo'))
    import run_demo
    run_demo.run()


def from_tum(gt_path, est_path):
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    gt = np.loadtxt(gt_path)[:, 1:4]
    est = np.loadtxt(est_path)[:, 1:4]
    n = min(len(gt), len(est)); gt, est = gt[:n], est[:n]
    est = est - est[0] + gt[0]
    figdir = os.path.join(os.path.dirname(__file__), '..', 'docs', 'figures')
    os.makedirs(figdir, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(gt[:, 0], gt[:, 1], 'k--', lw=1, label='ground truth')
    ax.plot(est[:, 0], est[:, 1], color='#2E5F84', lw=1.8, label='VIO estimate')
    ax.set_title('Estimate vs ground truth (measured)'); ax.legend(); ax.grid(True, lw=0.4)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); fig.tight_layout()
    fig.savefig(os.path.join(figdir, 'measured_trajectory.png'), dpi=140)
    print('wrote measured_trajectory.png  (run eval/compute_drift.py for the numbers)')


def main():
    if '--offline' in sys.argv or len(sys.argv) == 1:
        offline()
    elif len(sys.argv) == 3:
        from_tum(sys.argv[1], sys.argv[2])
    else:
        print('usage: make_report_figures.py [--offline | groundtruth.txt estimate.txt]')


if __name__ == '__main__':
    main()
