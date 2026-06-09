"""
A.E.T.H.E.R  degradation_manager.

Subscribes the integrity verdict and re-publishes the discrete navigation mode
(NOMINAL / DEGRADED / INERTIAL / RE_ACQUIRE) on a clean, latched topic for the
cockpit and for audit logs. The continuous R(D) logic lives in the integrity
core; this node owns the explicit, logged mode transitions and an audit trail.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from aether_msgs.msg import IntegrityState, NavMode

_NAMES = {0: 'NOMINAL', 1: 'DEGRADED', 2: 'INERTIAL', 3: 'RE_ACQUIRE'}


class DegradationManager(Node):
    def __init__(self):
        super().__init__('degradation_manager')
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_mode = self.create_publisher(NavMode, '/nav/mode', latched)
        self.create_subscription(IntegrityState, '/nav/integrity_state', self.on_state, 10)
        self.last = None
        self.get_logger().info('A.E.T.H.E.R degradation_manager up.')

    def on_state(self, msg: IntegrityState):
        m = msg.mode.mode
        if m != self.last:
            self.get_logger().warning(
                f'NAV MODE -> {_NAMES.get(m, m)}  (trust={msg.trust:.2f}, '
                f'HPL={msg.protection.horizontal_pl:.2f} m, '
                f'outage={msg.time_since_aiding_s:.2f} s)')
            self.last = m
        self.pub_mode.publish(msg.mode)


def main(args=None):
    rclpy.init(args=args)
    node = DegradationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
