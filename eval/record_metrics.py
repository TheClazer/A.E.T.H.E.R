#!/usr/bin/env python3
"""
A.E.T.H.E.R measured-metrics recorder — standalone rclpy script (U12).

Subscribes to the live integrity stack and writes ONE CSV row per
/nav/integrity_bound message, so a single demo run yields the measured
report (eval/make_measured_figures.py turns the CSV into figures).

Run (after sourcing ROS2 + the workspace):
    python3 eval/record_metrics.py [results/metrics.csv]

SIGINT-safe: every row is flushed as it is written, so Ctrl-C (or the
scripts/run_measured_report.sh teardown) never loses data.
"""
from __future__ import annotations
import csv
import math
import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String

from aether_msgs.msg import ProtectionLevel

FIELDS = ['t', 'est_x', 'est_y', 'est_z', 'gt_x', 'gt_y', 'gt_z', 'herr',
          'hpl_op', 'hpl_dalc', 'ellipse_yaw', 'trust', 'state', 'nees',
          'sep', 'latency', 'fault']


class MetricsRecorder(Node):
    """Caches the latest value on every contract topic; the integrity bound
    is the row clock — each /nav/integrity_bound message emits one CSV row."""

    def __init__(self, csv_path: str):
        super().__init__('metrics_recorder')
        self._est = None                  # (x, y, z) from /ov_msckf/odomimu
        self._gt = None                   # (x, y, z) from /aether/ground_truth
        self._trust = float('nan')
        self._state = ''
        self._nees = float('nan')
        self._sep = float('nan')
        self._latency = float('nan')
        self._fault = False
        self.rows = 0

        parent = os.path.dirname(os.path.abspath(csv_path))
        os.makedirs(parent, exist_ok=True)
        self._fh = open(csv_path, 'w', newline='')
        self._csv = csv.writer(self._fh)
        self._csv.writerow(FIELDS)
        self._fh.flush()

        self.create_subscription(Odometry, '/ov_msckf/odomimu', self._on_est, 20)
        self.create_subscription(Odometry, '/aether/ground_truth', self._on_gt, 20)
        self.create_subscription(Float32, '/nav/trust', self._on_trust, 10)
        self.create_subscription(String, '/nav/state', self._on_state, 10)
        self.create_subscription(Float32, '/nav/nees', self._on_nees, 10)
        self.create_subscription(Float32, '/nav/solution_separation', self._on_sep, 10)
        self.create_subscription(Float32, '/nav/detection_latency', self._on_lat, 10)
        self.create_subscription(Bool, '/fault/active', self._on_fault, 10)
        self.create_subscription(ProtectionLevel, '/nav/integrity_bound', self._on_bound, 20)
        self.get_logger().info('recording -> %s  (Ctrl-C to stop)' % csv_path)

    # ---- latest-value caches ------------------------------------------------
    def _on_est(self, m):
        p = m.pose.pose.position
        self._est = (p.x, p.y, p.z)

    def _on_gt(self, m):
        p = m.pose.pose.position
        self._gt = (p.x, p.y, p.z)

    def _on_trust(self, m):
        self._trust = float(m.data)

    def _on_state(self, m):
        self._state = str(m.data)

    def _on_nees(self, m):
        self._nees = float(m.data)

    def _on_sep(self, m):
        self._sep = float(m.data)

    def _on_lat(self, m):
        self._latency = float(m.data)

    def _on_fault(self, m):
        self._fault = bool(m.data)

    # ---- row clock ----------------------------------------------------------
    def _on_bound(self, m):
        if self._est is None:
            return  # no estimate yet — nothing meaningful to log
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        ex, ey, ez = self._est
        gx, gy, gz = self._gt if self._gt is not None else (float('nan'),) * 3
        herr = math.hypot(ex - gx, ey - gy) if self._gt is not None else float('nan')
        # ellipse_yaw is new in this msg rev; tolerate a stale build gracefully
        yaw = float(getattr(m, 'ellipse_yaw', float('nan')))
        self._csv.writerow([
            '%.4f' % t,
            '%.4f' % ex, '%.4f' % ey, '%.4f' % ez,
            '%.4f' % gx, '%.4f' % gy, '%.4f' % gz,
            '%.4f' % herr,
            '%.4f' % m.pl_operational, '%.4f' % m.pl_dal_c, '%.6f' % yaw,
            '%.4f' % self._trust, self._state, '%.4f' % self._nees,
            '%.4f' % self._sep, '%.4f' % self._latency,
            1 if self._fault else 0,
        ])
        self._fh.flush()  # SIGINT-safe: a kill mid-run still leaves a valid CSV
        self.rows += 1

    def close(self):
        try:
            self._fh.flush()
            self._fh.close()
        except Exception:
            pass


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join('results', 'metrics.csv')
    rclpy.init()
    node = MetricsRecorder(csv_path)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        print('record_metrics: wrote %d rows -> %s' % (node.rows, csv_path))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
