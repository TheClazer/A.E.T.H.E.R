#!/usr/bin/env bash
# A.E.T.H.E.R SCENARIO TOUR — a narrated, scripted pass through every fault class,
# run against whichever scene is already up (replay or live3d). Watch MISSION
# CONTROL / RViz while it runs. Each beat prints what to look at.
#
#   ./scripts/demo_tour.sh           # full tour (~2.5 min)
set -e
source /opt/ros/lyrical/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null || true
[ -f install/setup.bash ] && source install/setup.bash

say() { echo; echo "════════════════════════════════════════════════════════"; echo "  $1"; echo "════════════════════════════════════════════════════════"; }
kill_cam()   { ros2 service call /kill_camera aether_msgs/srv/KillCamera "{enable: $1}" > /dev/null; }
set_bool()   { ros2 service call "$1" std_srvs/srv/SetBool "{data: $2}" > /dev/null; }

say "SCENE NOMINAL — watch: trust GREEN, tight ellipse hugging the truth dot"
sleep 15

say "FAULT 1 · CAMERA KILL — the camera pane goes black; trust collapses;
  state -> INERTIAL within ~0.5 s; the bound BLOOMS but keeps the truth inside;
  the naive grey ghost stays small — and LOSES the truth (turns red)"
kill_cam true
sleep 20

say "RECOVERY — vision restored: drift decays in ~1.5 s, ellipse contracts, GREEN"
kill_cam false
sleep 12

say "FAULT 2 · IMU BIAS — the published IMU lies (+0.3 m/s2) while vision looks
  healthy. The feature count CANNOT catch this; watch SOL SEP rise until the
  solution-separation test trips (trust capped, bound flagged invalid)"
set_bool /inject/imu_bias true
sleep 22

say "RECOVERY — bias cleared"
set_bool /inject/imu_bias false
sleep 10

say "FAULT 3 · FEATURE STARVATION — a feature-poor stretch (~30 feats): state
  DEGRADED, trust AMBER, bound widens moderately. Degradation, not panic."
set_bool /inject/feature_starvation true
sleep 18

say "RECOVERY — features back"
set_bool /inject/feature_starvation false
sleep 10

say "LAYERED AIDING · UWB ON during a camera kill — HANA's layered-modality story:
  vision dies, but the UWB aid keeps the error (and the bound) clamped ~0.3 m"
kill_cam true
sleep 8
set_bool /uwb/enable true
sleep 15

say "FULL RECOVERY — camera + UWB restored to nominal"
kill_cam false
set_bool /uwb/enable false
sleep 8

say "TOUR COMPLETE — every fault class detected, bounded, and recovered.
  'When the camera dies, our drift bound still covers the true position
   95%+ of the time — and we know within half a second.'"
