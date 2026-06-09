"""
A.E.T.H.E.R  replay_node  —  the guaranteed live demo, with EMERGENT physics.

Publishes a representative stereo-inertial VIO trajectory on the SAME topics
OpenVINS would, so the REAL integrity stack (integrity_monitor + degradation_manager
+ health_cockpit) runs live in ROS2 with zero dependence on OpenVINS/Gazebo
converging. This is the demo that cannot fail on stage.

  publishes : /ov_msckf/odomimu (nav_msgs/Odometry, pose + 6x6 covariance)
              /ov_msckf/points_msckf (sensor_msgs/PointCloud2, width = feature count)
              /aether/ground_truth (nav_msgs/Odometry)
              /imu/data (sensor_msgs/Imu, 50 Hz, synthetic, gravity included)
              /fault/active (std_msgs/Bool, 20 Hz, true while ANY fault is injected)
  services  : /kill_camera (aether_msgs/KillCamera)        vision loss -> INERTIAL
              /inject/imu_bias (std_srvs/SetBool)          0.3 m/s^2 x-bias on /imu/data only
              /inject/feature_starvation (std_srvs/SetBool) features -> ~30 -> DEGRADED
              /uwb/enable (std_srvs/SetBool)               pseudo position fixes @ 1 Hz

Drift is EMERGENT, not constructed: a seeded per-axis [p_err, v_err, bias] truth-error
model integrates IMU-grade noise (accel white noise + bias random walk). During an
outage the ESTIMATE dead-reckons on that realized random walk while the PUBLISHED
covariance is the analytic propagation of the same stochastic model — so whether the
truth stays inside k*sigma is a statistical outcome (~95% at k=2.45), never rigged.
The synthetic /imu/data stream is driven by the SAME noise realization, which makes a
downstream IMU-only solution-separation channel statistically consistent with the
published drift.

Honesty: the VIO estimate here is a controlled trajectory, not OpenVINS on real data —
it exercises the integrity LOGIC live. Real accuracy numbers come from the separate
OpenVINS-on-EuRoC run (see docs/RUNBOOK). Use this for the live kill-camera beat.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header
from std_srvs.srv import SetBool
from aether_msgs.srv import KillCamera

RATE = 20.0                  # odometry / model rate (Hz)
DT = 1.0 / RATE
IMU_RATE = 50.0              # synthetic IMU rate (Hz)
SPEED = 1.5                  # m/s forward
SIG_NOM = np.array([0.06, 0.06, 0.04])   # nominal 1-sigma position error (m)

# IMU-grade error model (drives BOTH the drift realization and /imu/data)
S_A = 0.05                   # accel white noise, m/s^2 per 20 Hz step
S_RW = 0.01                  # accel bias random walk, m/s^2 / sqrt(s)
TAU_FIX = 0.4                # vision-correction time constant (s): errors decay to
                             # ~2% within ~1.5 s of vision being (re)acquired
LAM = float(np.exp(-DT / TAU_FIX))       # per-step error decay while vision aids
GRAVITY = 9.8                # +z in published accel; consumers subtract [0, 0, 9.8]
IMU_BIAS_X = 0.3             # injected sensor-only x accel bias (m/s^2)
UWB_SIGMA = 0.30             # UWB pseudo-fix 1-sigma clamp (m)
UWB_GAIN = 0.3               # fraction of position error surviving a UWB fix

# Analytic propagation of the SAME 3-state model the drift realization integrates.
# State per axis: x = [p_err, v_err, b];  x+ = F x + noise.
F = np.array([[1.0, DT, 0.5 * DT * DT],
              [0.0, 1.0, DT],
              [0.0, 0.0, 1.0]])
_G_A = np.array([0.5 * DT * DT, DT, 0.0])         # accel white noise injection
Q = (S_A ** 2) * np.outer(_G_A, _G_A)
Q[2, 2] += (S_RW ** 2) * DT                        # bias random-walk increment


class ReplayNode(Node):
    def __init__(self):
        super().__init__('aether_replay')
        self.pub_odom = self.create_publisher(Odometry, '/ov_msckf/odomimu', 20)
        self.pub_pts = self.create_publisher(PointCloud2, '/ov_msckf/points_msckf', 10)
        self.pub_gt = self.create_publisher(Odometry, '/aether/ground_truth', 20)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 50)
        self.pub_fault = self.create_publisher(Bool, '/fault/active', 10)

        self.create_service(KillCamera, '/kill_camera', self.on_kill)
        self.create_service(SetBool, '/inject/imu_bias', self.on_imu_bias)
        self.create_service(SetBool, '/inject/feature_starvation', self.on_starvation)
        self.create_service(SetBool, '/uwb/enable', self.on_uwb)

        # Fault flags
        self.killed = False          # vision dead -> dead-reckon on realized drift
        self.imu_biased = False      # sensor-only bias on PUBLISHED /imu/data
        self.starved = False         # low features + inflated sigma -> DEGRADED
        self.uwb_on = False          # layered-modality aiding stub

        # Seeded truth-error realization: per-axis [p_err, v_err, b] and its
        # analytic covariance. Same rng drives drift, IMU noise and feature counts
        # so every run is reproducible.
        self.rng = np.random.default_rng(7)
        self.err = np.zeros((3, 3))      # rows = axes x/y/z, cols = [p, v, b]
        self.P = np.zeros((3, 3, 3))     # per-axis 3x3 analytic covariance
        self.a_err = np.zeros(3)         # latest realized accel error (b + w_a)

        self.t0 = self.now()
        self.create_timer(DT, self.tick)
        self.create_timer(1.0 / IMU_RATE, self.tick_imu)
        self.create_timer(1.0, self.tick_uwb)
        self.get_logger().info(
            'A.E.T.H.E.R replay up (emergent drift, seed=7). Faults: /kill_camera, '
            '/inject/imu_bias, /inject/feature_starvation. Aiding: /uwb/enable.')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ services
    def on_kill(self, req, resp):
        self.killed = bool(req.enable)
        self.get_logger().warning(
            'VISION ' + ('LOST (dead-reckoning)' if self.killed else 'RESTORED'))
        resp.acknowledged = True
        return resp

    def on_imu_bias(self, req, resp):
        self.imu_biased = bool(req.data)
        self.get_logger().warning(
            'IMU BIAS FAULT ' + ('ON (+%.1f m/s^2 on published x accel; VIO estimate '
                                 'unaffected -> solution separation must catch it)'
                                 % IMU_BIAS_X if self.imu_biased else 'OFF'))
        resp.success = True
        resp.message = 'imu_bias=' + str(self.imu_biased)
        return resp

    def on_starvation(self, req, resp):
        self.starved = bool(req.data)
        self.get_logger().warning(
            'FEATURE STARVATION ' + ('ON (~30 features, sigma x1.5 -> DEGRADED)'
                                     if self.starved else 'OFF'))
        resp.success = True
        resp.message = 'feature_starvation=' + str(self.starved)
        return resp

    def on_uwb(self, req, resp):
        self.uwb_on = bool(req.data)
        self.get_logger().warning('UWB AIDING ' + ('ENABLED' if self.uwb_on else 'DISABLED'))
        resp.success = True
        resp.message = 'uwb=' + str(self.uwb_on)
        return resp

    # ------------------------------------------------------------------ truth path
    def _truth(self, t):
        """Forward at 1.5 m/s with a gentle weave; returns (pos, vel, accel)."""
        pos = np.array([SPEED * t, 0.6 * np.sin(t / 3.0), 1.2])
        vel = np.array([SPEED, 0.2 * np.cos(t / 3.0), 0.0])
        acc = np.array([0.0, -(0.6 / 9.0) * np.sin(t / 3.0), 0.0])
        return pos, vel, acc

    # ------------------------------------------------------------------ 20 Hz model
    def tick(self):
        t = self.now() - self.t0
        gt, vel_truth, _ = self._truth(t)

        # One step of the seeded truth-error realization (always running, so the
        # same stochastic history feeds /imu/data and any later outage).
        w_rw = self.rng.normal(0.0, S_RW * np.sqrt(DT), 3)
        w_a = self.rng.normal(0.0, S_A, 3)
        self.err[:, 2] += w_rw                       # bias random walk
        self.a_err = self.err[:, 2] + w_a            # realized accel error
        self.err[:, 1] += self.a_err * DT            # v_err
        self.err[:, 0] += self.err[:, 1] * DT        # p_err

        sig_floor = SIG_NOM * (1.5 if self.starved else 1.0)

        if self.killed:
            # VISION DEAD: the estimate dead-reckons on the REALIZED random walk;
            # the published covariance is the ANALYTIC propagation of the same
            # model (F, Q above). Coverage is therefore EMERGENT — the truth sits
            # inside k*sigma ~95% of the time at k=2.45 because the statistics
            # say so, not because the drift was constructed to fit the bound.
            for ax in range(3):
                self.P[ax] = F @ self.P[ax] @ F.T + Q
            est = gt + self.err[:, 0]
            n_feat = 0
        else:
            # Vision aiding: corrections continuously reset the error states
            # toward 0 (and contract the covariance) with time constant TAU_FIX,
            # so after a restore the drift decays away over ~1.5 s.
            self.err *= LAM
            self.P *= LAM * LAM
            est = gt + self.err[:, 0] + self.rng.normal(0.0, sig_floor)
            if self.starved:
                n_feat = int(max(5, self.rng.normal(30, 5)))
            else:
                n_feat = int(max(70, self.rng.normal(120, 6)))

        var_pub = self.P[:, 0, 0] + sig_floor ** 2   # nominal-noise floor

        stamp = self.get_clock().now().to_msg()
        # Twist carries the world-frame velocity: the downstream solution-
        # separation channel anchors its dead reckoning on it, so the estimate
        # gets truth velocity + the realized velocity error of the drift model.
        self.pub_odom.publish(self._odom(stamp, est, var_pub, vel_truth + self.err[:, 1]))
        self.pub_gt.publish(self._odom(stamp, gt, np.full(3, 1e-6), vel_truth))
        self.pub_pts.publish(self._cloud(stamp, n_feat))
        self.pub_fault.publish(Bool(data=bool(self.killed or self.imu_biased or self.starved)))

    # ------------------------------------------------------------------ 50 Hz IMU
    def tick_imu(self):
        """Synthetic IMU: truth acceleration + the SAME realized accel error that
        drives the drift (zero-order hold between 20 Hz model steps) + gravity.
        World-frame aligned, orientation identity."""
        t = self.now() - self.t0
        _, _, acc = self._truth(t)
        a = acc + self.a_err
        a[2] += GRAVITY                              # consumers subtract [0, 0, 9.8]
        if self.imu_biased:
            a[0] += IMU_BIAS_X                       # sensor-only fault: published
                                                     # IMU lies, VIO estimate does not
        m = Imu()
        m.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='imu')
        m.orientation.w = 1.0
        m.linear_acceleration.x = float(a[0])
        m.linear_acceleration.y = float(a[1])
        m.linear_acceleration.z = float(a[2])
        g = self.rng.normal(0.0, 1e-3, 3)
        m.angular_velocity.x = float(g[0])
        m.angular_velocity.y = float(g[1])
        m.angular_velocity.z = float(g[2])
        self.pub_imu.publish(m)

    # ------------------------------------------------------------------ 1 Hz UWB
    def tick_uwb(self):
        # UWB aiding stub: range-beacon fusion placeholder (HANA layered-modality demo).
        # A pseudo position-fix every 1.0 s pulls the realized position error in and
        # clamps the analytic position variance to (0.30 m)^2.
        if not self.uwb_on:
            return
        self.err[:, 0] *= UWB_GAIN
        for ax in range(3):
            p00 = self.P[ax][0, 0]
            if p00 > UWB_SIGMA ** 2:
                # Scale row/col 0 (congruence with diag(r,1,1)) so P stays PSD.
                r = float(np.sqrt(UWB_SIGMA ** 2 / p00))
                self.P[ax][0, :] *= r
                self.P[ax][:, 0] *= r

    # ------------------------------------------------------------------ messages
    def _odom(self, stamp, p, var, vel):
        m = Odometry()
        m.header = Header(stamp=stamp, frame_id='odom')
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x = float(p[0])
        m.pose.pose.position.y = float(p[1])
        m.pose.pose.position.z = float(p[2])
        m.pose.pose.orientation.w = 1.0
        m.twist.twist.linear.x = float(vel[0])
        m.twist.twist.linear.y = float(vel[1])
        m.twist.twist.linear.z = float(vel[2])
        cov = [0.0] * 36
        cov[0] = float(var[0])    # P[0,0] (position x) -- ROS order [x,y,z,rx,ry,rz]
        cov[7] = float(var[1])    # P[1,1]
        cov[14] = float(var[2])   # P[2,2]
        m.pose.covariance = cov
        return m

    def _cloud(self, stamp, n):
        hdr = Header(stamp=stamp, frame_id='odom')
        pts = [[float(i % 5), float(i // 5 % 5), 1.0] for i in range(max(0, n))]
        return point_cloud2.create_cloud_xyz32(hdr, pts)


def main(args=None):
    rclpy.init(args=args)
    node = ReplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
