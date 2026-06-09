"""
A.E.T.H.E.R  health_cockpit  (RViz Marker publisher).

Renders the breathing integrity ellipse as an RViz ``visualization_msgs/Marker``
centred on the VIO estimate, oriented by the principal-axis yaw of the
horizontal error ellipse (``ProtectionLevel.ellipse_yaw``), coloured by trust
(green/amber/red). Also drops a small marker on the true position (sim) so the
audience sees the truth staying inside the bound.

NEW: the NAIVE ghost — the overconfident baseline. While no fault is active it
shadows the VIO estimate with a tiny frozen protection level (nominal radius,
never widens). The moment ``/fault/active`` goes true it dead-reckons at its
last velocity and KEEPS the tiny ellipse. When the truth escapes that ellipse
the ghost turns bright red: a system confidently lost. Published on
``/viz/naive_bound`` and ``/viz/naive_estimate``, with TEXT_VIEW_FACING
verdict labels (and live detection latency) on ``/viz/labels``.

For the live dashboard see ``mission_hud.py`` (console script ``mission_hud``).
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker
from aether_msgs.msg import ProtectionLevel

# Frozen horizontal PL radius (m) of the naive baseline: the nominal-conditions
# bound it never widens, no matter what happens to the sensors.
NAIVE_PL_RADIUS = 0.21


def trust_rgb(trust):
    if trust >= 0.8:
        return (0.18, 0.49, 0.31)   # green
    if trust >= 0.4:
        return (0.79, 0.54, 0.07)   # amber
    return (0.77, 0.16, 0.10)       # red


def yaw_quat(yaw):
    """Quaternion (x, y, z, w) for a pure yaw rotation."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Cockpit(Node):
    def __init__(self):
        super().__init__('health_cockpit')
        self.est = None
        self.gt = None
        self.trust = 1.0
        self.pl = None
        self.detection_latency = None
        self.fault_active = False
        self.fault_seen = False          # a fault has been injected at least once

        # Naive ghost state: position + last-known velocity (m/s, odom frame).
        self.naive_pos = None            # [x, y, z]
        self.naive_vel = [0.0, 0.0, 0.0]
        self._prev_odom = None           # (x, y, z, t_sec) for finite-difference velocity
        self._last_tick = None           # monotonic node-clock seconds, for dead-reckon dt

        self.create_subscription(Odometry, '/ov_msckf/odomimu', self.on_odom, 20)
        self.create_subscription(Odometry, '/aether/ground_truth', self.on_gt, 20)
        self.create_subscription(Float32, '/nav/trust', self.on_trust, 10)
        self.create_subscription(ProtectionLevel, '/nav/integrity_bound', self.on_pl, 10)
        self.create_subscription(String, '/nav/state', self.on_state, 10)
        self.create_subscription(Bool, '/fault/active', self.on_fault, 10)
        self.create_subscription(Float32, '/nav/detection_latency', self.on_latency, 10)

        self.pub_ellipse = self.create_publisher(Marker, '/viz/integrity_bound', 10)
        self.pub_truth = self.create_publisher(Marker, '/viz/truth', 10)
        self.pub_naive_bound = self.create_publisher(Marker, '/viz/naive_bound', 10)
        self.pub_naive_est = self.create_publisher(Marker, '/viz/naive_estimate', 10)
        self.pub_labels = self.create_publisher(Marker, '/viz/labels', 10)

        self.create_timer(0.05, self.tick)
        self.get_logger().info('A.E.T.H.E.R health_cockpit up. RViz: /viz/integrity_bound '
                               '+ /viz/naive_bound + /viz/labels')

    # ------------------------------------------------------------------ subs
    def on_odom(self, msg):
        self.est = msg.pose.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self.fault_active:
            # Ghost shadows the VIO estimate, and remembers its velocity
            # (lightly low-passed finite difference) for the dead-reckon phase.
            self.naive_pos = [self.est.x, self.est.y, self.est.z]
            if self._prev_odom is not None:
                px, py, pz, pt = self._prev_odom
                dt = t - pt
                if 1e-4 < dt < 1.0:
                    a = 0.2
                    self.naive_vel[0] += a * ((self.est.x - px) / dt - self.naive_vel[0])
                    self.naive_vel[1] += a * ((self.est.y - py) / dt - self.naive_vel[1])
                    self.naive_vel[2] += a * ((self.est.z - pz) / dt - self.naive_vel[2])
            self._prev_odom = (self.est.x, self.est.y, self.est.z, t)

    def on_gt(self, msg):
        self.gt = msg.pose.pose.position

    def on_trust(self, msg):
        self.trust = msg.data

    def on_pl(self, msg):
        self.pl = msg

    def on_state(self, msg):
        pass

    def on_fault(self, msg):
        if msg.data:
            self.fault_seen = True
        if self.fault_active and not msg.data:
            self._prev_odom = None       # restart velocity estimate cleanly
        self.fault_active = msg.data

    def on_latency(self, msg):
        self.detection_latency = msg.data

    # --------------------------------------------------------------- helpers
    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _marker(self, ns, mid, mtype):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _label(self, mid, text, pos, rgba, z_off=0.6):
        m = self._marker('aether_labels', mid, Marker.TEXT_VIEW_FACING)
        m.text = text
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = pos[2] + z_off
        m.scale.z = 0.18
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        self.pub_labels.publish(m)

    def _truth_in_aether_bound(self):
        """Is the true position inside the AETHER horizontal error ellipse?"""
        if self.gt is None or self.est is None or self.pl is None:
            return True
        a = max(self.pl.semi_axes[0], 1e-6)
        b = max(self.pl.semi_axes[1], 1e-6)
        yaw = getattr(self.pl, 'ellipse_yaw', 0.0)
        dx = self.gt.x - self.est.x
        dy = self.gt.y - self.est.y
        c, s = math.cos(-yaw), math.sin(-yaw)
        u = c * dx - s * dy
        v = s * dx + c * dy
        return (u / a) ** 2 + (v / b) ** 2 <= 1.0

    def _truth_in_naive_bound(self):
        """Is the true position inside the naive ghost's frozen circle?"""
        if self.gt is None or self.naive_pos is None:
            return True
        dx = self.gt.x - self.naive_pos[0]
        dy = self.gt.y - self.naive_pos[1]
        return math.hypot(dx, dy) <= NAIVE_PL_RADIUS

    # ------------------------------------------------------------------ tick
    def tick(self):
        now = self._now_sec()
        dt = 0.05 if self._last_tick is None else max(0.0, min(now - self._last_tick, 0.5))
        self._last_tick = now

        # Naive ghost: constant-velocity dead-reckon while the fault is active.
        if self.fault_active and self.naive_pos is not None:
            self.naive_pos[0] += self.naive_vel[0] * dt
            self.naive_pos[1] += self.naive_vel[1] * dt
            self.naive_pos[2] += self.naive_vel[2] * dt

        if self.est is None or self.pl is None:
            return

        # --- AETHER breathing ellipse, oriented by the principal-axis yaw ---
        r, g, b = trust_rgb(self.trust)
        m = self._marker('aether', 0, Marker.CYLINDER)   # flat disc = horizontal PL
        m.pose.position = self.est
        qx, qy, qz, qw = yaw_quat(getattr(self.pl, 'ellipse_yaw', 0.0))
        m.pose.orientation.x = qx
        m.pose.orientation.y = qy
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw
        m.scale.x = max(self.pl.semi_axes[0] * 2.0, 0.05)
        m.scale.y = max(self.pl.semi_axes[1] * 2.0, 0.05)
        m.scale.z = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.35
        self.pub_ellipse.publish(m)

        # --- truth dot ---
        if self.gt is not None:
            t = self._marker('aether_truth', 1, Marker.SPHERE)
            t.pose.position = self.gt
            t.scale.x = t.scale.y = t.scale.z = 0.12
            t.color.r, t.color.g, t.color.b, t.color.a = 0.77, 0.16, 0.10, 1.0
            self.pub_truth.publish(t)

        # --- naive ghost: frozen tiny ellipse + small grey estimate sphere ---
        naive_lost = False
        if self.naive_pos is not None:
            naive_lost = not self._truth_in_naive_bound()

            nb = self._marker('naive', 2, Marker.CYLINDER)
            nb.pose.position.x = self.naive_pos[0]
            nb.pose.position.y = self.naive_pos[1]
            nb.pose.position.z = self.naive_pos[2]
            nb.scale.x = nb.scale.y = NAIVE_PL_RADIUS * 2.0
            nb.scale.z = 0.02
            if naive_lost:
                nb.color.r, nb.color.g, nb.color.b, nb.color.a = 1.0, 0.05, 0.05, 0.6
            else:
                nb.color.r, nb.color.g, nb.color.b, nb.color.a = 0.44, 0.50, 0.56, 0.25
            self.pub_naive_bound.publish(nb)

            ne = self._marker('naive', 3, Marker.SPHERE)
            ne.pose.position.x = self.naive_pos[0]
            ne.pose.position.y = self.naive_pos[1]
            ne.pose.position.z = self.naive_pos[2]
            ne.scale.x = ne.scale.y = ne.scale.z = 0.08
            ne.color.r, ne.color.g, ne.color.b, ne.color.a = 0.55, 0.58, 0.62, 0.9
            self.pub_naive_est.publish(ne)

        # --- verdict labels ---
        if self._truth_in_aether_bound():
            self._label(10, 'AETHER: TRUTH IN BOUND',
                        (self.est.x, self.est.y, self.est.z),
                        (0.30, 0.85, 0.45, 1.0))
        else:
            self._label(10, 'AETHER: TRUTH OUT OF BOUND',
                        (self.est.x, self.est.y, self.est.z),
                        (1.0, 0.25, 0.20, 1.0))

        if self.naive_pos is not None:
            if naive_lost:
                self._label(11, 'NAIVE: OVERCONFIDENT - TRUTH LOST',
                            (self.naive_pos[0], self.naive_pos[1], self.naive_pos[2]),
                            (1.0, 0.10, 0.10, 1.0), z_off=0.85)
            else:
                self._label(11, 'NAIVE: TRUTH IN BOUND',
                            (self.naive_pos[0], self.naive_pos[1], self.naive_pos[2]),
                            (0.65, 0.68, 0.72, 1.0), z_off=0.85)

        if self.fault_seen and self.detection_latency is not None and self.detection_latency > 0.0:
            self._label(12, 'detected in %.2f s' % self.detection_latency,
                        (self.est.x, self.est.y, self.est.z),
                        (0.95, 0.95, 0.95, 1.0), z_off=1.1)


def main(args=None):
    rclpy.init(args=args)
    node = Cockpit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
