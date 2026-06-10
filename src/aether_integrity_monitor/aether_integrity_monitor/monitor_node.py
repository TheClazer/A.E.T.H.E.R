"""
A.E.T.H.E.R  integrity_monitor  (the novel layer).

Consumes ONLY what stock OpenVINS already publishes — pose+covariance on
``/ov_msckf/odomimu`` and a tracked-feature count — and emits a live navigation
integrity verdict:

  * /nav/integrity_state       (aether_msgs/IntegrityState)  — the full bus
  * /nav/integrity_bound       (aether_msgs/ProtectionLevel) — the breathing bound
  * /nav/trust                 (std_msgs/Float32)            — continuous 0..1
  * /nav/state                 (std_msgs/String)             — NOMINAL/.../RE_ACQUIRE
  * /nav/nees                  (std_msgs/Float32)            — consistency vs GT (sim)
  * /nav/solution_separation   (std_msgs/Float32)            — IMU-vs-VIO sep (m)

It also runs a solution-separation test: a simple IMU dead-reckoning channel
(re-anchored to the VIO solution every SEP_WINDOW_S) is compared against the
VIO pose; a persistent, covariance-normalized disagreement flags an
IMU/vision inconsistency even when the feature count looks healthy.

No C++ fork required — this is the de-risked live demo path. The math lives in
``core.py`` (a mirror of offline_demo/aether_core.py, validated off-Ubuntu).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float32, String

from aether_msgs.msg import IntegrityState, ProtectionLevel, NavMode
from . import core


class IntegrityMonitor(Node):
    def __init__(self):
        super().__init__('integrity_monitor')
        # parameters (overridable from config/integrity.yaml — replay-rig scales —
        # or config/integrity_ov.yaml — real-OpenVINS scales: ~10-30 features per
        # MSCKF update and a covariance that grows on a loop-closure-free traverse)
        self.declare_parameter('k_operational', core.K_OP)
        self.declare_parameter('k_dal_c', core.K_DALC)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('n_feat_nominal', float(core.N_NOMINAL))
        self.declare_parameter('tr_nominal', float(core.TR_NOMINAL))
        self.declare_parameter('n_feat_degraded', float(core.N_DEGRADED))
        self.declare_parameter('n_feat_inertial', float(core.N_INERTIAL))
        self.declare_parameter('n_feat_reacquire', float(core.N_REACQUIRE))
        self.declare_parameter('debounce_s', float(core.DEBOUNCE_S))
        self.declare_parameter('tr_rate_max', 0.0)
        # OpenVINS marginalizes MSCKF features in BURSTS (many updates carry 0),
        # so the health signal is the max over a short window, not the raw width.
        # 0.0 = use the raw count (replay rig publishes a steady count).
        self.declare_parameter('n_feat_window_s', 0.0)
        self.k_op = self.get_parameter('k_operational').value
        self.k_dalc = self.get_parameter('k_dal_c').value
        self.n_nominal = float(self.get_parameter('n_feat_nominal').value)
        self.tr_nominal = float(self.get_parameter('tr_nominal').value)
        self.n_degraded = float(self.get_parameter('n_feat_degraded').value)
        self.n_feat_window_s = float(self.get_parameter('n_feat_window_s').value)
        self._feat_hist = []           # (t, n) samples inside the smoothing window

        self.sub_odom = self.create_subscription(Odometry, '/ov_msckf/odomimu', self.on_odom, 20)
        self.sub_feat = self.create_subscription(PointCloud2, '/ov_msckf/points_msckf', self.on_feat, 10)
        self.sub_gt = self.create_subscription(Odometry, '/aether/ground_truth', self.on_gt, 20)
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.on_imu, 50)

        self.pub_state = self.create_publisher(IntegrityState, '/nav/integrity_state', 10)
        self.pub_bound = self.create_publisher(ProtectionLevel, '/nav/integrity_bound', 10)
        self.pub_trust = self.create_publisher(Float32, '/nav/trust', 10)
        self.pub_str = self.create_publisher(String, '/nav/state', 10)
        self.pub_nees = self.create_publisher(Float32, '/nav/nees', 10)
        self.pub_sep = self.create_publisher(Float32, '/nav/solution_separation', 10)

        self.P_pos = np.eye(3) * 1e-3
        self.x_est = None
        self.x_vel = None
        self.x_gt = None
        self.n_feat = self.n_nominal
        self.tr_prev = None
        self.t_last_aiding = self.now()
        self.sm = core.DegradationStateMachine(
            debounce_s=float(self.get_parameter('debounce_s').value),
            n_inertial=float(self.get_parameter('n_feat_inertial').value),
            n_reacquire=float(self.get_parameter('n_feat_reacquire').value),
            n_degraded=self.n_degraded,
            tr_rate_max=float(self.get_parameter('tr_rate_max').value))

        # solution-separation dead-reckoning channel (re-anchored to VIO)
        self.imu_p = None          # dead-reckoned position (odom frame)
        self.imu_v = None          # dead-reckoned velocity (odom frame)
        self.imu_t_prev = None     # last IMU sample time (s)
        self.sep_anchor_t = None   # time of last channel re-anchor (s)
        self.separation = 0.0      # raw horizontal separation (m)
        self.sep_exceed_t = None   # time normalized sep first exceeded threshold
        self.sep_confirm_t = None  # last time the persist condition confirmed
        self.sep_fault = False     # persistence-gated inconsistency flag

        period = 1.0 / float(self.get_parameter('rate_hz').value)
        self.create_timer(period, self.tick)
        self.get_logger().info('A.E.T.H.E.R integrity_monitor up (Python proxy path).')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg: Odometry):
        self.P_pos = core.position_cov(msg.pose.covariance)
        p = msg.pose.pose.position
        self.x_est = np.array([p.x, p.y, p.z])
        v = msg.twist.twist.linear
        self.x_vel = np.array([v.x, v.y, v.z])

    def on_feat(self, msg: PointCloud2):
        n = float(msg.width)
        if self.n_feat_window_s > 0.0:
            t = self.now()
            self._feat_hist.append((t, n))
            self._feat_hist = [(ts, v) for ts, v in self._feat_hist
                               if t - ts <= self.n_feat_window_s]
            self.n_feat = max(v for _, v in self._feat_hist)
        else:
            self.n_feat = n
        if self.n_feat >= self.n_degraded:
            self.t_last_aiding = self.now()

    def on_gt(self, msg: Odometry):
        p = msg.pose.pose.position
        self.x_gt = np.array([p.x, p.y, p.z])

    def on_imu(self, msg: Imu):
        """Dead-reckoning channel for the solution-separation test.

        Integrates accel minus gravity [0, 0, 9.8] in the odom frame.
        Simplifying assumption (matches the replay/sim bridge): the IMU reports
        world-frame-aligned acceleration and the odom twist is world-frame, so
        no orientation rotation is applied here. The channel is re-anchored to
        the latest VIO solution every SEP_WINDOW_S (sliding window), so only
        short-horizon disagreement accumulates.
        """
        if self.x_est is None:
            return
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if (self.imu_p is None or self.sep_anchor_t is None
                or (t - self.sep_anchor_t) >= core.SEP_WINDOW_S):
            self.imu_p = self.x_est.copy()
            self.imu_v = self.x_vel.copy() if self.x_vel is not None else np.zeros(3)
            self.sep_anchor_t = t
            self.imu_t_prev = t
            return
        dt = t - self.imu_t_prev
        self.imu_t_prev = t
        if dt <= 0.0 or dt > 0.5:   # rewind or gap — skip this step
            return
        a = np.array([msg.linear_acceleration.x,
                      msg.linear_acceleration.y,
                      msg.linear_acceleration.z]) - np.array([0.0, 0.0, core.GRAVITY])
        self.imu_v = self.imu_v + a * dt
        self.imu_p = self.imu_p + self.imu_v * dt

    def tick(self):
        tr = float(np.trace(self.P_pos))
        tr_rate = 0.0 if self.tr_prev is None else (tr - self.tr_prev) * 20.0
        self.tr_prev = tr
        t = self.now()

        pl_op = core.protection_level(self.P_pos, self.k_op)
        semi_major, semi_minor, ell_yaw = core.horizontal_ellipse(self.P_pos, self.k_op)
        hpl = semi_major                      # HPL = k * sqrt(lambda_max)
        hpl_dalc = core.horizontal_pl(self.P_pos, self.k_dalc)
        trust = core.trust_score(self.n_feat, tr,
                                 n_nominal=self.n_nominal, tr_nominal=self.tr_nominal)
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

        # solution separation: VIO vs IMU dead-reckoning, normalized by the
        # worst-direction horizontal sigma (floored). A persistent exceedance is
        # an IMU/vision inconsistency — caught even when feature count is high.
        sqrt_lam_max = max(hpl / max(self.k_op, 1e-9), core.SEP_SIGMA_FLOOR)
        if self.imu_p is not None and self.x_est is not None:
            d = self.x_est - self.imu_p
            self.separation = float(np.hypot(d[0], d[1]))
        sep_norm = self.separation / sqrt_lam_max
        if sep_norm > core.SEP_NORM_THRESH:
            if self.sep_exceed_t is None:
                self.sep_exceed_t = t
            if (t - self.sep_exceed_t) > core.SEP_PERSIST_S:
                if not self.sep_fault:
                    self.get_logger().warning(
                        f'solution-separation fault: sep={self.separation:.2f} m '
                        f'({sep_norm:.1f} sigma) — capping trust, bound invalid')
                self.sep_fault = True
                self.sep_confirm_t = t
        else:
            self.sep_exceed_t = None
            # hold the verdict across the periodic re-anchor: separation resets
            # to ~0 every SEP_WINDOW_S even while the underlying fault persists
            if self.sep_fault and (self.sep_confirm_t is None
                                   or (t - self.sep_confirm_t) > core.SEP_WINDOW_S):
                self.sep_fault = False
        if self.sep_fault:
            trust = min(trust, core.SEP_TRUST_CAP)
            bound_valid = False
        # A confirmed separation fault is a detected integrity event even when
        # the feature count looks healthy (e.g. an IMU bias): escalate the
        # published state so the degradation manager's detection-latency clock
        # sees the departure from NOMINAL.
        if self.sep_fault and state == 'NOMINAL':
            state = 'DEGRADED'
        self.pub_sep.publish(Float32(data=float(self.separation)))

        pl = ProtectionLevel()
        pl.header.stamp = self.get_clock().now().to_msg()
        pl.header.frame_id = 'odom'
        pl.pl_operational = float(hpl)
        pl.pl_dal_c = float(hpl_dalc)
        pl.horizontal_pl = float(hpl)
        pl.covariance_trace = tr
        pl.semi_axes = [float(semi_major), float(semi_minor), float(pl_op[2])]
        pl.ellipse_yaw = float(ell_yaw)
        self.pub_bound.publish(pl)

        mode = NavMode()
        mode.mode = {'NOMINAL': 0, 'DEGRADED': 1, 'INERTIAL': 2, 'RE_ACQUIRE': 3}[state]
        st = IntegrityState()
        st.header = pl.header
        st.protection = pl
        st.observability_index = float(core.observability_index(self.P_pos))
        st.nees = float(nees_val)
        st.trust = float(trust)
        st.bound_valid = bool(bound_valid)
        st.time_since_aiding_s = float(t - self.t_last_aiding)
        st.mode = mode
        st.solution_separation = float(self.separation)
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
