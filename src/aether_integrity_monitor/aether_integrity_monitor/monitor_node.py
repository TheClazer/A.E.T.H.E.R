"""
A.E.T.H.E.R  integrity_monitor  (the novel layer).

Consumes ONLY what stock OpenVINS already publishes — pose+covariance on
``/ov_msckf/odomimu`` and a tracked-feature count — and emits a live navigation
integrity verdict:

  * /nav/integrity_state  (aether_msgs/IntegrityState)  — the full bus
  * /nav/integrity_bound  (aether_msgs/ProtectionLevel) — the breathing bound
  * /nav/trust            (std_msgs/Float32)            — continuous 0..1
  * /nav/state            (std_msgs/String)             — NOMINAL/.../RE_ACQUIRE
  * /nav/nees             (std_msgs/Float32)            — consistency vs GT (sim)

No C++ fork required — this is the de-risked live demo path. The math lives in
``core.py`` (a mirror of offline_demo/aether_core.py, validated off-Ubuntu).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String

from aether_msgs.msg import IntegrityState, ProtectionLevel, NavMode
from . import core


class IntegrityMonitor(Node):
    def __init__(self):
        super().__init__('integrity_monitor')
        # parameters (overridable from config/integrity.yaml)
        self.declare_parameter('k_operational', core.K_OP)
        self.declare_parameter('k_dal_c', core.K_DALC)
        self.declare_parameter('rate_hz', 20.0)
        self.k_op = self.get_parameter('k_operational').value
        self.k_dalc = self.get_parameter('k_dal_c').value

        self.sub_odom = self.create_subscription(Odometry, '/ov_msckf/odomimu', self.on_odom, 20)
        self.sub_feat = self.create_subscription(PointCloud2, '/ov_msckf/points_msckf', self.on_feat, 10)
        self.sub_gt = self.create_subscription(Odometry, '/aether/ground_truth', self.on_gt, 20)

        self.pub_state = self.create_publisher(IntegrityState, '/nav/integrity_state', 10)
        self.pub_bound = self.create_publisher(ProtectionLevel, '/nav/integrity_bound', 10)
        self.pub_trust = self.create_publisher(Float32, '/nav/trust', 10)
        self.pub_str = self.create_publisher(String, '/nav/state', 10)
        self.pub_nees = self.create_publisher(Float32, '/nav/nees', 10)

        self.P_pos = np.eye(3) * 1e-3
        self.x_est = None
        self.x_gt = None
        self.n_feat = core.N_NOMINAL
        self.tr_prev = None
        self.t_last_aiding = self.now()
        self.sm = core.DegradationStateMachine()

        period = 1.0 / float(self.get_parameter('rate_hz').value)
        self.create_timer(period, self.tick)
        self.get_logger().info('A.E.T.H.E.R integrity_monitor up (Python proxy path).')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg: Odometry):
        self.P_pos = core.position_cov(msg.pose.covariance)
        p = msg.pose.pose.position
        self.x_est = np.array([p.x, p.y, p.z])

    def on_feat(self, msg: PointCloud2):
        self.n_feat = float(msg.width)
        if self.n_feat >= core.N_DEGRADED:
            self.t_last_aiding = self.now()

    def on_gt(self, msg: Odometry):
        p = msg.pose.pose.position
        self.x_gt = np.array([p.x, p.y, p.z])

    def tick(self):
        tr = float(np.trace(self.P_pos))
        tr_rate = 0.0 if self.tr_prev is None else (tr - self.tr_prev) * 20.0
        self.tr_prev = tr
        t = self.now()

        pl_op = core.protection_level(self.P_pos, self.k_op)
        hpl = core.horizontal_pl(self.P_pos, self.k_op)
        hpl_dalc = core.horizontal_pl(self.P_pos, self.k_dalc)
        trust = core.trust_score(self.n_feat, tr)
        state = self.sm.update(self.n_feat, tr_rate, t)

        nees_val = -1.0
        bound_valid = True
        if self.x_est is not None and self.x_gt is not None:
            try:
                nees_val = core.nees(self.x_est - self.x_gt, self.P_pos)
                lo, hi = core.nees_gate()
                bound_valid = (lo <= nees_val <= hi)
                self.pub_nees.publish(Float32(data=float(nees_val)))
            except np.linalg.LinAlgError:
                pass

        pl = ProtectionLevel()
        pl.header.stamp = self.get_clock().now().to_msg()
        pl.header.frame_id = 'odom'
        pl.pl_operational = float(hpl)
        pl.pl_dal_c = float(hpl_dalc)
        pl.horizontal_pl = float(hpl)
        pl.covariance_trace = tr
        pl.semi_axes = [float(v) for v in pl_op]
        self.pub_bound.publish(pl)

        mode = NavMode()
        mode.mode = {'NOMINAL': 0, 'DEGRADED': 1, 'INERTIAL': 2, 'RE_ACQUIRE': 3}[state]
        st = IntegrityState()
        st.header = pl.header
        st.protection = pl
        st.observability_index = float(min(self.n_feat / core.N_NOMINAL, 1.0))
        st.nees = float(nees_val)
        st.trust = float(trust)
        st.bound_valid = bool(bound_valid)
        st.time_since_aiding_s = float(t - self.t_last_aiding)
        st.mode = mode
        self.pub_state.publish(st)

        self.pub_trust.publish(Float32(data=float(trust)))
        self.pub_str.publish(String(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = IntegrityMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
