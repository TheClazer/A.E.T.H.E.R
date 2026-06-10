#!/usr/bin/env python3
"""Record the integrity verdict + truth into a CSV while the chain plays.

    python3 integrity_stream_to_csv.py out.csv

Columns: t, trust, state, hpl_op, hpl_dalc, est_x..z, gt_x..z, herr.
Used by the full-integrity chain to prove the bound covers the REAL OpenVINS
error. Ctrl-C / SIGINT flushes and exits.
"""
import sys
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String

from aether_msgs.msg import ProtectionLevel


class Recorder(Node):
    def __init__(self, path):
        super().__init__('integrity_csv_recorder')
        self.f = open(path, 'w')
        self.f.write('t,trust,state,hpl_op,hpl_dalc,est_x,est_y,est_z,gt_x,gt_y,gt_z,herr\n')
        self.trust = float('nan')
        self.state = ''
        self.pl_op = float('nan')
        self.pl_dalc = float('nan')
        self.est = None
        self.gt = None
        self.n = 0
        self.create_subscription(Float32, '/nav/trust', lambda m: setattr(self, 'trust', m.data), 10)
        self.create_subscription(String, '/nav/state', lambda m: setattr(self, 'state', m.data), 10)
        self.create_subscription(ProtectionLevel, '/nav/integrity_bound', self.on_pl, 10)
        # estimate: namespaced on stock launches, ROOT-namespaced on the upstream
        # ov_msckf build in the chain container — subscribe both, rows fire on either
        self.create_subscription(Odometry, '/ov_msckf/odomimu', self.on_est, 20)
        self.create_subscription(Odometry, '/odomimu', self.on_est, 20)
        self.create_subscription(Odometry, '/aether/ground_truth', self.on_gt, 20)

    def on_pl(self, m):
        self.pl_op, self.pl_dalc = m.pl_operational, m.pl_dal_c

    def on_gt(self, m):
        p = m.pose.pose.position
        self.gt = (p.x, p.y, p.z)

    def on_est(self, m):
        p = m.pose.pose.position
        self.est = (p.x, p.y, p.z)
        if self.gt is None:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        # NOTE: est is in the VIO world frame (arbitrary yaw); herr here is only
        # meaningful AFTER offline alignment — the CSV keeps both raw tracks and
        # the analysis script aligns before computing coverage.
        herr = math.nan
        self.f.write(f'{t:.6f},{self.trust:.4f},{self.state},{self.pl_op:.4f},{self.pl_dalc:.4f},'
                     f'{self.est[0]:.4f},{self.est[1]:.4f},{self.est[2]:.4f},'
                     f'{self.gt[0]:.4f},{self.gt[1]:.4f},{self.gt[2]:.4f},{herr}\n')
        self.n += 1


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'integrity_chain.csv'
    rclpy.init()
    node = Recorder(out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.flush()
        node.f.close()
        print(f'wrote {node.n} rows -> {out}')
        node.destroy_node()


if __name__ == '__main__':
    main()
