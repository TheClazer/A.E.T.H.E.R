#!/usr/bin/env python3
"""Measure |accel| std in 1-s sliding windows over the takeoff segment of the
golden bag — tells us exactly what init_imu_thresh would trip (OpenVINS static
init compares the accel-norm std of the newest window against the threshold)."""
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

r = SequentialReader()
r.open(StorageOptions(uri='/root/aether/bags/tunnel_run', storage_id=''),
       ConverterOptions('cdr', 'cdr'))
acc, t0 = [], None
while r.has_next():
    tn, data, _ = r.read_next()
    if tn != '/imu/data':
        continue
    m = deserialize_message(data, Imu)
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if t0 is None:
        t0 = t
    tt = t - t0
    if tt > 30.0:
        break
    a = m.linear_acceleration
    acc.append((tt, np.sqrt(a.x**2 + a.y**2 + a.z**2)))
a = np.array(acc)
print('samples:', len(a), ' span: %.1f s' % a[-1, 0])
print('%6s  %10s  %10s' % ('t(s)', 'std|a|', 'mean|a|'))
for w0 in np.arange(0.0, 28.0, 1.0):
    w = a[(a[:, 0] >= w0) & (a[:, 0] < w0 + 1.0)]
    if len(w) < 10:
        continue
    print('%6.1f  %10.4f  %10.3f' % (w0, w[:, 1].std(), w[:, 1].mean()))
