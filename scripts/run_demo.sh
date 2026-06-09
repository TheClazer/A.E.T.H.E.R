#!/usr/bin/env bash
# A.E.T.H.E.R kill-the-camera demo helper.
#  - with no arg: prints the demo beat sequence.
#  - `kill`     : blanks the stereo stream (trust should flip RED < 0.5 s, bound blooms).
#  - `restore`  : restores the stream (bound contracts, trust returns GREEN).
set -euo pipefail
source /opt/ros/humble/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true

case "${1:-help}" in
  kill)    ros2 service call /kill_camera aether_msgs/srv/KillCamera "{enable: true}" ;;
  restore) ros2 service call /kill_camera aether_msgs/srv/KillCamera "{enable: false}" ;;
  *)
    cat <<'TXT'
A.E.T.H.E.R demo beat sheet (run aether.launch.py + rviz2 -d config/aether.rviz first):
  0-8s   NOMINAL   trust GREEN, bound tight, estimate tracks the moving truth dot
  8s     ./scripts/run_demo.sh kill      -> camera black; stock VIO starts to drift
  8-9s   INTEGRITY trust -> RED in < 0.5s; mode INERTIAL; the bound starts to bloom
  9-28s  COAST     bound blooms ~1.0 m, chasing & containing the moving truth (TRUTH in BOUND)
  28s    ./scripts/run_demo.sh restore   -> vision back; bound contracts; trust GREEN
Watch /nav/trust /nav/state /nav/integrity_bound ; HUD: streamlit run src/aether_health_cockpit/aether_health_cockpit/streamlit_app.py
"When the camera dies, our drift bound still covers the true position 95%+ of the time -- and we know within half a second."
TXT
    ;;
esac
