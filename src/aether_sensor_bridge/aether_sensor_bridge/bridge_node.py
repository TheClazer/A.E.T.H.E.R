"""
A.E.T.H.E.R  sensor_bridge.

Remaps the raw Gazebo (or EuRoC-replay) stereo + IMU topics onto the topic
contract OpenVINS expects, and serves the demo's ``/kill_camera`` fault-injection
service (blanks the image stream so the integrity layer can be shown reacting).

  in : /camera/left/image_raw  /camera/right/image_raw  /imu/data
  out: /camera/left/image      /camera/right/image      /imu0
  srv: /kill_camera (aether_msgs/KillCamera)  enable=true -> stop forwarding images
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from aether_msgs.srv import KillCamera


class SensorBridge(Node):
    def __init__(self):
        super().__init__('sensor_bridge')
        self.camera_dead = False

        self.pub_left = self.create_publisher(Image, '/camera/left/image', 10)
        self.pub_right = self.create_publisher(Image, '/camera/right/image', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu0', 50)

        self.create_subscription(Image, '/camera/left/image_raw', self.on_left, 10)
        self.create_subscription(Image, '/camera/right/image_raw', self.on_right, 10)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 50)

        self.srv = self.create_service(KillCamera, '/kill_camera', self.on_kill)
        self.get_logger().info('A.E.T.H.E.R sensor_bridge up. /kill_camera ready.')

    def on_left(self, msg):
        if not self.camera_dead:
            self.pub_left.publish(msg)

    def on_right(self, msg):
        if not self.camera_dead:
            self.pub_right.publish(msg)

    def on_imu(self, msg):
        self.pub_imu.publish(msg)        # IMU is the spine — never killed

    def on_kill(self, req, resp):
        self.camera_dead = bool(req.enable)
        state = 'DEAD' if self.camera_dead else 'LIVE'
        self.get_logger().warning(f'/kill_camera -> camera {state}')
        resp.acknowledged = True
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
