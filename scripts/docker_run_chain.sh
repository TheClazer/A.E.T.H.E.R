#!/usr/bin/env bash
# A.E.T.H.E.R full chain, container side (runs INSIDE aether/openvins:jazzy):
#   golden rosbag (our Gazebo tunnel flight) -> OpenVINS ov_msckf (stereo MSCKF)
#   -> TUM trajectories -> KITTI %-drift vs ground truth.
# Invoke from WSL:
#   docker run --rm -v /mnt/d/Work/AETHER:/aether aether/openvins:jazzy \
#     bash /aether/scripts/docker_run_chain.sh [bag_dir(default /aether/bags/tunnel_run)]
set -o pipefail
source /opt/ros/jazzy/setup.bash
source /ws_ov/install/setup.bash

BAG="${1:-/aether/bags/tunnel_run}"
OUT=/aether/results
mkdir -p "$OUT"
CFG=/tmp/ovcfg
# configs are parsed relative to the estimator file; strip CRLF defensively
mkdir -p "$CFG"
for f in estimator_config kalibr_imu_chain kalibr_imucam_chain; do
  tr -d '\r' < "/aether/sim/openvins_config/$f.yaml" > "$CFG/$f.yaml"
done

echo "=== [1/3] start OpenVINS + TUM streamers ==="
ros2 run ov_msckf run_subscribe_msckf "$CFG/estimator_config.yaml" > "$OUT/ov_msckf.log" 2>&1 &
OV=$!
python3 /aether/eval/odom_stream_to_tum.py /ov_msckf/odomimu "$OUT/est.txt" > /dev/null 2>&1 &
S1=$!
python3 /aether/eval/odom_stream_to_tum.py /aether/ground_truth "$OUT/gt.txt" > /dev/null 2>&1 &
S2=$!
sleep 6

echo "=== [2/3] play the golden bag (rate 1.0, trimmed past the hover tail) ==="
# flight profile ends ~150 s into the bag; the rest is a parked hover — skip it
PLAY_SECS="${PLAY_SECS:-175}"
ros2 bag play "$BAG" --playback-duration "$PLAY_SECS" > /dev/null 2>&1 \
  || ros2 bag play "$BAG" --storage mcap --playback-duration "$PLAY_SECS" > /dev/null 2>&1
sleep 4

echo "=== [3/3] evaluate ==="
kill -INT $S1 $S2 2>/dev/null; sleep 2
kill -9 $OV $S1 $S2 2>/dev/null
wc -l "$OUT/est.txt" "$OUT/gt.txt" 2>/dev/null
python3 /aether/eval/compute_drift.py "$OUT/gt.txt" "$OUT/est.txt" | tee "$OUT/drift_report.txt"
echo "=== ov log tail ==="
tail -5 "$OUT/ov_msckf.log"
echo "CHAIN_DONE"
