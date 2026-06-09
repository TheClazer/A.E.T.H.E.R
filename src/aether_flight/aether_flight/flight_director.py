"""
A.E.T.H.E.R  flight_director.

Open-loop trajectory generator for the kinematic sensor rig (U2). Publishes
geometry_msgs/Twist on /model/aether_drone/cmd_vel at 20 Hz; the ros_gz bridge
forwards it to the VelocityControl system on the drone model.

Profile (gentle IMU excitation, no yaw ambiguity — zero angular rates):
  vx = 1.5 m/s                  steady advance down the corridor
  vy = 0.35 * sin(0.4 * t)      slow lateral weave
  vz = 0.08 * sin(0.9 * t)      small vertical bob
After 'duration' seconds (param, default 150 s) it commands zero velocity and
keeps publishing zeros so the rig parks instead of coasting.

Time base is the node clock, so it follows /clock when use_sim_time is set.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class FlightDirector(Node):
    def __init__(self):
        super().__init__('flight_director')
        self.declare_parameter('duration', 150.0)
        self.duration = float(self.get_parameter('duration').value)

        self.pub = self.create_publisher(Twist, '/model/aether_drone/cmd_vel', 10)
        self.t0 = None            # set on first non-zero clock sample
        self.stopped = False
        self.timer = self.create_timer(0.05, self.tick)   # 20 Hz
        self.get_logger().info(
            f'A.E.T.H.E.R flight_director up: {self.duration:.0f} s profile '
            'on /model/aether_drone/cmd_vel')

    def tick(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return                # sim time not flowing yet — wait for /clock
        if self.t0 is None:
            self.t0 = now
        t = (now - self.t0).nanoseconds * 1e-9

        cmd = Twist()             # angular stays zero — no yaw ambiguity
        if t < self.duration:
            cmd.linear.x = 1.5
            cmd.linear.y = 0.35 * math.sin(0.4 * t)
            cmd.linear.z = 0.08 * math.sin(0.9 * t)
        elif not self.stopped:
            self.stopped = True
            self.get_logger().info(f'profile complete at t={t:.1f} s — holding zero velocity')
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FlightDirector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
