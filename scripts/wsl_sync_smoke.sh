#!/usr/bin/env bash
# Sync the changed sim/flight files into ~/aether, rebuild aether_flight,
# run a short record, and report (a) GT motion extent and (b) IMU liveness
# during hold/punch/cruise — the physics-real actuation check.
set -o pipefail
source /opt/ros/lyrical/setup.bash
WS=~/aether
cp -r /mnt/d/Work/AETHER/sim/models/x3_aether "$WS/sim/models/"
cp /mnt/d/Work/AETHER/sim/worlds/tunnel.sdf "$WS/sim/worlds/"
cp /mnt/d/Work/AETHER/src/aether_flight/aether_flight/flight_director.py "$WS/src/aether_flight/aether_flight/"
cp /mnt/d/Work/AETHER/scripts/wsl_record_bag.sh "$WS/scripts/"
find "$WS/sim" "$WS/scripts" -type f \( -name '*.sdf' -o -name '*.config' -o -name '*.sh' \) -exec sed -i 's/\r$//' {} +
sed -i 's/\r$//' "$WS/src/aether_flight/aether_flight/flight_director.py"
cd "$WS"
source install/setup.bash
colcon build --packages-select aether_flight > /dev/null 2>&1 && echo FLIGHT_REBUILT
RECORD_SECS=40 bash scripts/wsl_record_bag.sh > /tmp/recx3.log 2>&1
grep -E "Duration|Count:" /tmp/recx3.log | head -7

python3 eval/odom_to_tum.py bags/tunnel_run /aether/ground_truth /tmp/gx.txt 2>/dev/null | tail -1
python3 - <<'EOF'
import numpy as np
d = np.loadtxt('/tmp/gx.txt'); t = d[:,0]-d[0,0]
print('GT: %d poses | %.1f s | x %.2f..%.2f | z %.2f..%.2f' %
      (len(d), t[-1], d[:,1].min(), d[:,1].max(), d[:,3].min(), d[:,3].max()))
EOF

python3 - <<'EOF'
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
import numpy as np
r = SequentialReader(); r.open(StorageOptions(uri='bags/tunnel_run', storage_id=''), ConverterOptions('cdr','cdr'))
acc=[]; t0=None
while r.has_next():
    tn, data, ts = r.read_next()
    if tn != '/imu/data': continue
    m = deserialize_message(data, Imu)
    t = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
    if t0 is None: t0 = t
    tt = t - t0
    if tt > 14.0: break
    acc.append((tt, m.linear_acceleration.x, m.linear_acceleration.z))
a = np.array(acc)
for lo,hi,label in [(1,4.5,'HOVER'),(5.0,7.2,'PUNCH'),(9,13,'CRUISE')]:
    w = a[(a[:,0]>=lo)&(a[:,0]<hi)]
    if len(w)==0: print(label, '(no samples)'); continue
    print('%6s  ax mean %+.3f std %.3f | az mean %+.3f std %.3f (n=%d)' %
          (label, w[:,1].mean(), w[:,1].std(), w[:,2].mean(), w[:,2].std(), len(w)))
EOF
echo "X3_SMOKE_DONE"
