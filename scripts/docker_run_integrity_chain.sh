#!/usr/bin/env bash
# THE COMPLETE A.E.T.H.E.R CHAIN, container side (aether/openvins:jazzy):
#   golden Gazebo bag -> OpenVINS stereo MSCKF -> integrity_monitor +
#   degradation_manager (the REAL integrity layer on the REAL VIO output)
#   -> CSV verdict + drift report.
#
# This build of OpenVINS publishes at the root namespace (/odomimu,
# /points_msckf), so the integrity nodes are launched with remappings.
#
# Invoke from WSL:
#   docker run --rm -v /mnt/d/Work/AETHER:/aether aether/openvins:jazzy \
#     bash -c 'tr -d "\r" < /aether/scripts/docker_run_integrity_chain.sh > /tmp/ic.sh && bash /tmp/ic.sh'
set -o pipefail
source /opt/ros/jazzy/setup.bash
source /ws_ov/install/setup.bash

BAG="${1:-/aether/bags/tunnel_run}"
OUT=/aether/results
PLAY_SECS="${PLAY_SECS:-165}"
mkdir -p "$OUT"

echo "=== [1/5] build the aether workspace (msgs + integrity nodes) ==="
WS=/tmp/aether_ws
mkdir -p "$WS/src"
cp -r /aether/src/aether_msgs /aether/src/aether_integrity_monitor \
      /aether/src/aether_degradation_manager /aether/src/aether_bringup "$WS/src/"
find "$WS/src" -type f -exec sed -i 's/\r$//' {} +
cd "$WS"
colcon build --packages-select aether_msgs --cmake-args -DBUILD_TESTING=OFF > /tmp/b1.log 2>&1 || { tail -5 /tmp/b1.log; exit 1; }
source install/setup.bash
colcon build --packages-select aether_integrity_monitor aether_degradation_manager > /tmp/b2.log 2>&1 || { tail -5 /tmp/b2.log; exit 1; }
source install/setup.bash
echo AETHER_WS_BUILT

echo "=== [2/5] start OpenVINS + integrity layer (remapped to root-ns topics) ==="
CFG=/tmp/ovcfg; mkdir -p "$CFG"
for f in estimator_config kalibr_imu_chain kalibr_imucam_chain; do
  tr -d '\r' < "/aether/sim/openvins_config/$f.yaml" > "$CFG/$f.yaml"
done
ros2 run ov_msckf run_subscribe_msckf "$CFG/estimator_config.yaml" > "$OUT/ov_msckf.log" 2>&1 &
OV=$!
tr -d '\r' < /aether/src/aether_bringup/config/integrity.yaml > /tmp/integrity.yaml
ros2 run aether_integrity_monitor monitor_node --ros-args -p use_sim_time:=false \
  --params-file /tmp/integrity.yaml \
  -r /ov_msckf/odomimu:=/odomimu -r /ov_msckf/points_msckf:=/points_msckf \
  > "$OUT/monitor_chain.log" 2>&1 &
MO=$!
ros2 run aether_degradation_manager manager_node > "$OUT/manager_chain.log" 2>&1 &
MG=$!
python3 /aether/eval/integrity_stream_to_csv.py "$OUT/integrity_chain.csv" > /tmp/rec.log 2>&1 &
RC=$!
python3 /aether/eval/odom_stream_to_tum.py /odomimu "$OUT/est.txt" > /dev/null 2>&1 &
S1=$!
python3 /aether/eval/odom_stream_to_tum.py /aether/ground_truth "$OUT/gt.txt" > /dev/null 2>&1 &
S2=$!
sleep 8

echo "=== [3/5] play the golden bag ==="
ros2 bag play "$BAG" --playback-duration "$PLAY_SECS" > /dev/null 2>&1
sleep 4

echo "=== [4/5] stop + drift report ==="
kill -INT $RC $S1 $S2 2>/dev/null; sleep 2
kill -9 $OV $MO $MG $RC $S1 $S2 2>/dev/null
python3 /aether/eval/compute_drift.py "$OUT/gt.txt" "$OUT/est.txt" | tee "$OUT/drift_report.txt"

echo "=== [5/5] integrity-on-real-VIO verdict ==="
python3 - <<'EOF'
import numpy as np
import csv

rows = list(csv.DictReader(open('/aether/results/integrity_chain.csv')))
print('csv rows:', len(rows))
if len(rows) < 100:
    print('INTEGRITY_CHAIN_INSUFFICIENT'); raise SystemExit
t   = np.array([float(r['t']) for r in rows])
est = np.array([[float(r['est_x']), float(r['est_y']), float(r['est_z'])] for r in rows])
gt  = np.array([[float(r['gt_x']),  float(r['gt_y']),  float(r['gt_z'])]  for r in rows])
pl  = np.array([float(r['hpl_op']) for r in rows])
tr  = np.array([float(r['trust']) for r in rows])
states = [r['state'] for r in rows]
# rigid alignment (VIO world yaw is arbitrary) then horizontal error
mu_e, mu_g = est.mean(0), gt.mean(0)
H = (est - mu_e).T @ (gt - mu_g)
U, _, Vt = np.linalg.svd(H)
S = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
R = Vt.T @ S @ U.T
est_a = (R @ est.T).T + (mu_g - R @ mu_e)
herr = np.linalg.norm((est_a - gt)[:, :2], axis=1)
ok = np.isfinite(pl) & (pl > 0)
cov = float(np.mean(herr[ok] <= pl[ok]) * 100.0)
print(f'IB coverage on REAL OpenVINS output (k_op=2.45): {cov:.1f}%  '
      f'(mean herr {herr[ok].mean():.3f} m, mean PL {pl[ok].mean():.3f} m)')
print(f'trust: mean {tr.mean():.2f}  min {tr.min():.2f}')
from collections import Counter
print('states:', dict(Counter(states)))
print('INTEGRITY_CHAIN_OK' if cov >= 90 else 'INTEGRITY_CHAIN_REVIEW')
EOF
echo "FULL_INTEGRITY_CHAIN_DONE"
