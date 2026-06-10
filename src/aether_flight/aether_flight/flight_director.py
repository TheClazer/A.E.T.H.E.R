"""
A.E.T.H.E.R  flight_director.

Open-loop trajectory generator for the kinematic sensor rig (U2). Publishes
geometry_msgs/Twist on /X3/gazebo/command/twist at 20 Hz; the ros_gz bridge
forwards it to the MulticopterVelocityControl system (body-frame velocity +
yaw-rate commands realized through REAL rotor forces, so the IMU senses true
accelerations - a hard VIO requirement).

Profile (VIO-initialization-friendly, then gentle excitation, zero angular rates):
  0 .. hold_s        : zero velocity — STATIC HOLD so OpenVINS static init can
                       average the IMU (it needs stillness, then a jerk)
  hold_s .. +1.0 s   : PUNCH — brisk ramp to cruise + a vertical kick (the
                       "accel jerk" the initializer detects)
  then               : vx = 1.5 m/s cruise, vy = 0.35*sin(0.4 t) weave,
                       vz = 0.08*sin(0.9 t) bob
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
        self.declare_parameter('hold', 5.0)       # static-hold seconds before motion
        self.duration = float(self.get_parameter('duration').value)
        self.hold = float(self.get_parameter('hold').value)

        self.pub = self.create_publisher(Twist, '/X3/gazebo/command/twist', 10)
        self.t0 = None            # set on first non-zero clock sample
        self.stopped = False
        self.timer = self.create_timer(0.05, self.tick)   # 20 Hz
        self.get_logger().info(
            f'A.E.T.H.E.R flight_director up: {self.duration:.0f} s profile '
            'on /X3/gazebo/command/twist')

    def tick(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return                # sim time not flowing yet — wait for /clock
        if self.t0 is None:
            self.t0 = now
        t = (now - self.t0).nanoseconds * 1e-9

        cmd = Twist()             # angular stays zero — no yaw ambiguity
        if t < self.hold:
            pass                  # hover hold: zero-velocity command (rotors carry weight,
                                  # IMU averages ~g) so OpenVINS static init can converge
        elif t < self.hold + 2.0:
            # the punch: a SHARP climb step (accel-norm std ~0.6 over the init
            # window — comfortably above init_imu_thresh 0.25) + forward ramp.
            # Net climb ~0.8 m to cruise altitude.
            s = t - self.hold
            cmd.linear.x = 1.5 * min(s / 0.4, 1.0)
            cmd.linear.z = 1.0 if s < 0.7 else (0.15 if s < 1.4 else 0.0)
        elif t < self.duration:
            tc = t - (self.hold + 2.0)
            cmd.linear.x = 1.5
            # COSINE weave: integral = (0.25/0.5)*sin -> y oscillates +/-0.5 m,
            # symmetric about the corridor centerline (the sin() form drifted
            # one-sided to +1.75 m and clipped the wall - the t=24s crash).
            cmd.linear.y = 0.25 * math.cos(0.5 * tc)
            cmd.linear.z = 0.08 * math.sin(0.9 * tc)
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
