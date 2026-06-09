#!/usr/bin/env python3
# Stream a live nav_msgs/Odometry topic to a TUM-format text file
# (timestamp tx ty tz qx qy qz qw), one line per message, flushed continuously.
# Used during rosbag playback so no second bag is needed:
#   python3 eval/odom_stream_to_tum.py /ov_msckf/odomimu est.txt
#   python3 eval/odom_stream_to_tum.py /aether/ground_truth gt.txt
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomToTum(Node):
    def __init__(self, topic, out_path):
        super().__init__('odom_stream_to_tum_' + topic.strip('/').replace('/', '_'))
        self.f = open(out_path, 'w')
        self.n = 0
        self.create_subscription(Odometry, topic, self.on_odom, 50)
        self.get_logger().info(f'streaming {topic} -> {out_path}')

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.f.write(f'{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} '
                     f'{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n')
        self.n += 1
        if self.n % 200 == 0:
            self.f.flush()


def main():
    if len(sys.argv) != 3:
        print('usage: odom_stream_to_tum.py <odom_topic> <out.txt>')
        sys.exit(1)
    rclpy.init()
    node = OdomToTum(sys.argv[1], sys.argv[2])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.flush()
        node.f.close()
        print(f'wrote {node.n} poses to {sys.argv[2]}')


if __name__ == '__main__':
    main()
