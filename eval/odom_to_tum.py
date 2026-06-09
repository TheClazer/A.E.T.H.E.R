#!/usr/bin/env python3
# Convert a recorded nav_msgs/Odometry topic in a ROS2 bag to TUM format
# (timestamp tx ty tz qx qy qz qw) for evo / ov_eval / compute_drift.py.
#
#   python eval/odom_to_tum.py <bag_dir> /ov_msckf/odomimu estimate.txt
#   python eval/odom_to_tum.py <bag_dir> /aether/ground_truth groundtruth.txt
#
# Runs on the Ubuntu box (needs rosbag2_py + ROS2). See docs/AETHER_MANUAL_STEPS.pdf.
import sys


def main():
    if len(sys.argv) != 4:
        print("usage: odom_to_tum.py <bag_dir> <odom_topic> <out.txt>"); sys.exit(1)
    bag, topic, out = sys.argv[1:4]
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = SequentialReader()
    # storage_id='' lets rosbag2 auto-detect (mcap on newer distros, sqlite3 on older)
    reader.open(StorageOptions(uri=bag, storage_id=''),
                ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_cls = get_message(types[topic])

    n = 0
    with open(out, 'w') as f:
        while reader.has_next():
            tname, data, _ = reader.read_next()
            if tname != topic:
                continue
            m = deserialize_message(data, msg_cls)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            f.write(f"{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
                    f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n")
            n += 1
    print(f"wrote {n} poses from {topic} -> {out}")


if __name__ == "__main__":
    main()
