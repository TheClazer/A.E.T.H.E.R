"""
A.E.T.H.E.R  flight_director.

Guidance node for the simulated mission (U2). Publishes geometry_msgs/Twist on
/X3/gazebo/command/twist at 20 Hz; the ros_gz bridge forwards it to the
MulticopterVelocityControl system (velocity + yaw-rate commands realized through
REAL rotor forces, so the IMU senses true accelerations - a hard VIO requirement).

GUIDANCE IS CLOSED-LOOP ON SIMULATOR GROUND TRUTH (/aether/ground_truth):
lateral/vertical position and yaw are tracked with feedforward + P feedback.
This is deliberate and honest: pure open-loop velocity commands drift unbounded
(any mm/s lateral bias or micro-yaw integrates; two recorded attempts hit the
corridor walls at t=24 s and t=40 s). The autopilot is NOT the system under
test - the VIO/integrity stack never sees ground truth; only the flight
controller does, exactly as a real autopilot would fly on its own nav solution.

Profile:
  0 .. hold_s   : parked (rotors carry no command) - OpenVINS static-init stillness
  hold .. +2 s  : PUNCH - sharp climb step (accel-norm std ~0.6 > init_imu_thresh
                  0.25, the init jerk) + forward ramp; climbs to ~0.8 m
  then          : 1.5 m/s cruise down the corridor with a +/-0.5 m sinusoidal
                  weave and a gentle altitude bob, position-locked to the
                  centerline reference; yaw held at 0
After 'duration' seconds it commands zero velocity and parks (hover-hold).

Time base is the node clock, so it follows /clock when use_sim_time is set.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FlightDirector(Node):
    def __init__(self):
        super().__init__('flight_director')
        self.declare_parameter('duration', 150.0)
        self.declare_parameter('hold', 5.0)        # parked seconds before motion
        self.declare_parameter('cruise_alt', 0.8)  # corridor cruise altitude (m)
        self.declare_parameter('kp_pos', 0.9)      # P gain, lateral/vertical hold
        self.declare_parameter('kp_yaw', 0.8)      # P gain, yaw hold
        # patrol mode (the judge demo): instead of a one-way 200 m mission that
        # ends parked, shuttle between x = patrol_min and x = patrol_max forever —
        # continuous visible motion that never leaves the textured tunnel.
        self.declare_parameter('patrol', False)
        self.declare_parameter('patrol_min', 5.0)
        self.declare_parameter('patrol_max', 90.0)
        self.duration = float(self.get_parameter('duration').value)
        self.hold = float(self.get_parameter('hold').value)
        self.alt = float(self.get_parameter('cruise_alt').value)
        self.kp = float(self.get_parameter('kp_pos').value)
        self.kyaw = float(self.get_parameter('kp_yaw').value)
        self.patrol = bool(self.get_parameter('patrol').value)
        self.patrol_min = float(self.get_parameter('patrol_min').value)
        self.patrol_max = float(self.get_parameter('patrol_max').value)
        self.patrol_dir = 1.0      # +1 = down-tunnel, -1 = homeward
        self.gt_x = None

        self.pub = self.create_publisher(Twist, '/X3/gazebo/command/twist', 10)
        self.create_subscription(Odometry, '/aether/ground_truth', self.on_gt, 20)
        self.gt = None            # (y, z, yaw) from the simulator
        self.t0 = None            # set on first non-zero clock sample
        self.stopped = False
        self.timer = self.create_timer(0.05, self.tick)   # 20 Hz
        self.get_logger().info(
            f'A.E.T.H.E.R flight_director up: {self.duration:.0f} s closed-loop '
            'profile on /X3/gazebo/command/twist')

    def on_gt(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.gt = (p.y, p.z, yaw)
        self.gt_x = p.x

    def tick(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return                # sim time not flowing yet — wait for /clock
        if self.t0 is None:
            self.t0 = now
        t = (now - self.t0).nanoseconds * 1e-9

        cmd = Twist()
        if t < self.hold:
            pass                  # parked: zeros (OpenVINS static init averages the IMU)
        elif t < self.hold + 2.0:
            # the punch: sharp climb step = the init jerk; forward ramp begins
            s = t - self.hold
            cmd.linear.x = 1.5 * min(s / 0.4, 1.0)
            if self.gt is not None and self.gt[1] >= self.alt:
                cmd.linear.z = 0.0
            else:
                cmd.linear.z = 1.0 if s < 0.7 else 0.3
        elif self.patrol or t < self.duration:
            tc = t - (self.hold + 2.0)
            # references: +/-0.5 m weave about the centerline, gentle altitude bob
            y_ref = 0.5 * math.sin(0.5 * tc)
            dy_ref = 0.25 * math.cos(0.5 * tc)            # feedforward = d(y_ref)/dt
            z_ref = self.alt + 0.10 * math.sin(0.9 * tc)
            dz_ref = 0.09 * math.cos(0.9 * tc)
            if self.patrol and self.gt_x is not None:
                # shuttle: flip direction at the patrol fence posts
                if self.patrol_dir > 0 and self.gt_x >= self.patrol_max:
                    self.patrol_dir = -1.0
                    self.get_logger().info('patrol: turning back (x=%.1f)' % self.gt_x)
                elif self.patrol_dir < 0 and self.gt_x <= self.patrol_min:
                    self.patrol_dir = 1.0
                    self.get_logger().info('patrol: heading out (x=%.1f)' % self.gt_x)
            cmd.linear.x = 1.5 * (self.patrol_dir if self.patrol else 1.0)
            if self.gt is not None:
                y, z, yaw = self.gt
                cmd.linear.y = clamp(dy_ref + self.kp * (y_ref - y), -0.7, 0.7)
                cmd.linear.z = clamp(dz_ref + self.kp * (z_ref - z), -0.6, 0.8)
                cmd.angular.z = clamp(-self.kyaw * yaw, -0.5, 0.5)
            else:                 # GT not up yet: feedforward only
                cmd.linear.y = dy_ref
                cmd.linear.z = dz_ref
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
