"""
A.E.T.H.E.R  degradation_manager.

Subscribes the integrity verdict and re-publishes the discrete navigation mode
(NOMINAL / DEGRADED / INERTIAL / RE_ACQUIRE) on a clean, latched topic for the
cockpit and for audit logs. The continuous R(D) logic lives in the integrity
core; this node owns the explicit, logged mode transitions and an audit trail.

It also measures DETECTION LATENCY: /fault/active (from the fault-injection
suite) marks the instant a fault is switched on; the first time the mode leaves
NOMINAL after that rising edge, the elapsed time is published on
/nav/detection_latency (Float32, seconds) and re-published at 1 Hz so the
cockpit can always show the last measured value.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Float32
from aether_msgs.msg import IntegrityState, NavMode

_NAMES = {0: 'NOMINAL', 1: 'DEGRADED', 2: 'INERTIAL', 3: 'RE_ACQUIRE'}


class DegradationManager(Node):
    def __init__(self):
        super().__init__('degradation_manager')
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_mode = self.create_publisher(NavMode, '/nav/mode', latched)
        self.pub_latency = self.create_publisher(Float32, '/nav/detection_latency', 10)
        self.create_subscription(IntegrityState, '/nav/integrity_state', self.on_state, 10)
        self.create_subscription(Bool, '/fault/active', self.on_fault, 10)
        self.last = None
        # Detection-latency bookkeeping (rising edge of /fault/active -> first
        # departure from NOMINAL).
        self.fault_prev = False
        self.t_fault = None      # stamp of the rising edge
        self.pending = False     # waiting for the mode to react to this fault
        self.latency = None      # last measured latency (s), re-published at 1 Hz
        self.create_timer(1.0, self.republish_latency)
        self.get_logger().info('A.E.T.H.E.R degradation_manager up.')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_fault(self, msg: Bool):
        if msg.data and not self.fault_prev:
            self.t_fault = self.now()
            self.pending = True
        elif not msg.data:
            self.pending = False
            self.t_fault = None
        self.fault_prev = bool(msg.data)

    def on_state(self, msg: IntegrityState):
        m = msg.mode.mode
        if m != self.last:
            self.get_logger().warning(
                f'NAV MODE -> {_NAMES.get(m, m)}  (trust={msg.trust:.2f}, '
                f'HPL={msg.protection.horizontal_pl:.2f} m, '
                f'outage={msg.time_since_aiding_s:.2f} s)')
            self.last = m
        if self.pending and m != NavMode.NOMINAL:
            self.latency = self.now() - self.t_fault
            self.pending = False
            self.pub_latency.publish(Float32(data=float(self.latency)))
            self.get_logger().warning(
                f'FAULT DETECTED: mode left NOMINAL {self.latency * 1000.0:.0f} ms '
                f'after injection (-> {_NAMES.get(m, m)})')
        self.pub_mode.publish(msg.mode)

    def republish_latency(self):
        if self.latency is not None:
            self.pub_latency.publish(Float32(data=float(self.latency)))


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
