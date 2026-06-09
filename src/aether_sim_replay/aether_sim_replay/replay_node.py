"""
A.E.T.H.E.R  replay_node  —  the guaranteed live demo.

Publishes a representative stereo-inertial VIO trajectory on the SAME topics
OpenVINS would, so the REAL integrity stack (integrity_monitor + degradation_manager
+ health_cockpit) runs live in ROS2 with zero dependence on OpenVINS/Gazebo
converging. This is the demo that cannot fail on stage.

  publishes : /ov_msckf/odomimu (nav_msgs/Odometry, pose+6x6 covariance)
              /ov_msckf/points_msckf (sensor_msgs/PointCloud2, width = feature count)
              /aether/ground_truth (nav_msgs/Odometry)
  service   : /kill_camera (aether_msgs/KillCamera)  enable=true -> simulate vision loss
              -> feature count collapses, covariance blooms, the bound breathes, truth
              stays inside it. enable=false -> recover.

Honesty: the VIO estimate here is a controlled trajectory, not OpenVINS on real data —
it exercises the integrity LOGIC live. Real accuracy numbers come from the separate
OpenVINS-on-EuRoC run (see docs/RUNBOOK). Use this for the live kill-camera beat.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from aether_msgs.srv import KillCamera

RATE = 20.0
SPEED = 1.5
SIG_NOM = np.array([0.06, 0.06, 0.04])
GROW = np.array([0.05, 0.05, 0.03])   # 1-sigma growth per second of outage


class ReplayNode(Node):
    def __init__(self):
        super().__init__('aether_replay')
        self.pub_odom = self.create_publisher(Odometry, '/ov_msckf/odomimu', 20)
        self.pub_pts = self.create_publisher(PointCloud2, '/ov_msckf/points_msckf', 10)
        self.pub_gt = self.create_publisher(Odometry, '/aether/ground_truth', 20)
        self.srv = self.create_service(KillCamera, '/kill_camera', self.on_kill)
        self.killed = False
        self.t0 = self.now()
        self.t_kill = None
        self.create_timer(1.0 / RATE, self.tick)
        self.get_logger().info('A.E.T.H.E.R replay up. Call /kill_camera to inject vision loss.')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_kill(self, req, resp):
        self.killed = bool(req.enable)
        self.t_kill = self.now() if self.killed else None
        self.get_logger().warn('VISION ' + ('LOST (dead-reckoning)' if self.killed else 'RESTORED'))
        resp.acknowledged = True
        return resp

    def tick(self):
        t = self.now() - self.t0
        # ground-truth path: forward at 1.5 m/s with a gentle weave
        gx, gy, gz = SPEED * t, 0.6 * np.sin(t / 3.0), 1.2
        gt = np.array([gx, gy, gz])

        if self.killed:
            dt = self.now() - self.t_kill
            n_feat = 0
            sigma = SIG_NOM + GROW * dt
            drift = 0.8 * sigma * np.array([1.0, -0.7, 0.5])   # truth stays ~0.8σ inside the bound
        else:
            n_feat = int(max(70, np.random.normal(120, 6)))
            sigma = SIG_NOM
            drift = sigma * np.random.normal(0, 0.5, 3)
        est = gt + drift

        stamp = self.get_clock().now().to_msg()
        self.pub_odom.publish(self._odom(stamp, est, sigma))
        self.pub_gt.publish(self._odom(stamp, gt, np.full(3, 1e-3)))
        self.pub_pts.publish(self._cloud(stamp, n_feat))

    def _odom(self, stamp, p, sigma):
        m = Odometry()
        m.header = Header(stamp=stamp, frame_id='odom')
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x = float(p[0])
        m.pose.pose.position.y = float(p[1])
        m.pose.pose.position.z = float(p[2])
        m.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0] = float(sigma[0] ** 2)   # P[0,0]  (position x)  -- ROS order [x,y,z,rx,ry,rz]
        cov[7] = float(sigma[1] ** 2)   # P[1,1]
        cov[14] = float(sigma[2] ** 2)  # P[2,2]
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
