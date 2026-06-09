"""
A.E.T.H.E.R  health_cockpit  (RViz Marker publisher).

Renders the breathing integrity ellipse as an RViz ``visualization_msgs/Marker``
centred on the VIO estimate, with semi-axes = the operational protection level,
coloured by trust (green/amber/red). Also drops a small marker on the true
position (sim) so the audience sees the truth staying inside the bound.

For the dashboard HUD see ``streamlit_app.py`` (run with ``streamlit run``).
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker
from aether_msgs.msg import ProtectionLevel


def trust_rgb(trust):
    if trust >= 0.8:
        return (0.18, 0.49, 0.31)   # green
    if trust >= 0.4:
        return (0.79, 0.54, 0.07)   # amber
    return (0.77, 0.16, 0.10)       # red


class Cockpit(Node):
    def __init__(self):
        super().__init__('health_cockpit')
        self.est = None
        self.trust = 1.0
        self.pl = None

        self.create_subscription(Odometry, '/ov_msckf/odomimu', self.on_odom, 20)
        self.create_subscription(Odometry, '/aether/ground_truth', self.on_gt, 20)
        self.create_subscription(Float32, '/nav/trust', self.on_trust, 10)
        self.create_subscription(ProtectionLevel, '/nav/integrity_bound', self.on_pl, 10)
        self.create_subscription(String, '/nav/state', self.on_state, 10)

        self.pub_ellipse = self.create_publisher(Marker, '/viz/integrity_bound', 10)
        self.pub_truth = self.create_publisher(Marker, '/viz/truth', 10)
        self.gt = None
        self.create_timer(0.05, self.tick)
        self.get_logger().info('A.E.T.H.E.R health_cockpit up. RViz: /viz/integrity_bound')

    def on_odom(self, msg):
        self.est = msg.pose.pose.position

    def on_gt(self, msg):
        self.gt = msg.pose.pose.position

    def on_trust(self, msg):
        self.trust = msg.data

    def on_pl(self, msg):
        self.pl = msg

    def on_state(self, msg):
        pass

    def tick(self):
        if self.est is None or self.pl is None:
            return
        r, g, b = trust_rgb(self.trust)
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'aether'
        m.id = 0
        m.type = Marker.CYLINDER          # flat disc = horizontal protection level
        m.action = Marker.ADD
        m.pose.position = self.est
        m.pose.orientation.w = 1.0
        sx = max(self.pl.semi_axes[0] * 2.0, 0.05)
        sy = max(self.pl.semi_axes[1] * 2.0, 0.05)
        m.scale.x, m.scale.y, m.scale.z = sx, sy, 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.35
        self.pub_ellipse.publish(m)

        if self.gt is not None:
            t = Marker()
            t.header = m.header
            t.ns = 'aether_truth'
            t.id = 1
            t.type = Marker.SPHERE
            t.action = Marker.ADD
            t.pose.position = self.gt
            t.pose.orientation.w = 1.0
            t.scale.x = t.scale.y = t.scale.z = 0.12
            t.color.r, t.color.g, t.color.b, t.color.a = 0.77, 0.16, 0.10, 1.0
            self.pub_truth.publish(t)


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
