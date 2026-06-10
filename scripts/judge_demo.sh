#!/usr/bin/env bash
# ╔══════════════════════════════════════════════════════════════════╗
# ║  A.E.T.H.E.R — JUDGE DEMO LAUNCHER (one command per scene)        ║
# ╚══════════════════════════════════════════════════════════════════╝
#
#   ./scripts/judge_demo.sh live3d   THE SHOWPIECE: Gazebo 3D drone patrolling the
#                                    tunnel + RViz (trails, breathing bound, camera
#                                    pane) + MISSION CONTROL with clickable faults
#   ./scripts/judge_demo.sh replay   the guaranteed rig (no Gazebo): same integrity
#                                    stack + MISSION CONTROL — bulletproof fallback
#   ./scripts/judge_demo.sh tour     narrated scripted pass through every fault
#                                    (run while live3d or replay is up)
#   ./scripts/judge_demo.sh chain    REAL OpenVINS on the recorded tunnel flight
#                                    (Docker) -> the 0.219%/202 m drift report
#   ./scripts/judge_demo.sh proof    print the measured results + open the figures
#
# First run builds the workspace automatically (~2 min).
set -e
cd "$(dirname "$0")/.."
source /opt/ros/lyrical/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash

# ── display bootstrap ───────────────────────────────────────────────
# The demo runs best in a ROOT WSL terminal (wsl -u root): Gazebo's sensor
# rendering crashes on the d3d12 EGL device that user sessions select, while
# root falls back to the software EGL path that recorded the golden bag.
# Root just needs to borrow the WSLg display:
if [ "$(id -u)" = "0" ] && [ -z "${DISPLAY:-}" ]; then
  export DISPLAY=:0 WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/run/user/1000
fi
if [ "$(id -u)" != "0" ] && [ "${1:-}" = "live3d" ]; then
  echo "NOTE: live3d needs the gz sensor-rendering path that only works as root"
  echo "      on this WSL setup. Start it from Windows with:   wsl -u root"
  echo "      then: cd $(pwd) && ./scripts/judge_demo.sh live3d"
  exit 1
fi

build_ws() {
  if [ ! -f install/setup.bash ] || [ src -nt install ]; then
    echo "[judge_demo] building the workspace (first run only) ..."
    colcon build --packages-select aether_msgs > /dev/null
    source install/setup.bash
    colcon build > /dev/null
  fi
  source install/setup.bash
}

case "${1:-help}" in
  live3d)
    build_ws
    cat <<'TXT'
┌────────────────────────────────────────────────────────────────────┐
│ LIVE 3D SCENE — what to watch                                      │
│  · Gazebo window: the X3 quad patrolling the 200 m textured tunnel │
│  · RViz: green truth trail vs blue estimate trail; breathing bound │
│    ellipse; grey naive ghost; the LEFT CAMERA pane                 │
│  · MISSION CONTROL: state banner + trust + clickable fault rail    │
│ Try it: click KILL CAMERA → the camera pane freezes/blanks, trust  │
│ collapses, the bound blooms — and the truth stays inside it.       │
│ (Estimate = validated VIO-class error model, labeled on screen.    │
│  Real-OpenVINS numbers: ./scripts/judge_demo.sh chain)             │
└────────────────────────────────────────────────────────────────────┘
TXT
    # The gz server runs STANDALONE (under `ros2 launch` its ogre-next texture
    # loader heap-crashes on this rig; the identical standalone start is solid).
    # Camera readiness is gated, with retries.
    export GZ_SIM_RESOURCE_PATH="$(pwd)/sim/models"
    pkill -9 -f "gz sim" 2>/dev/null; sleep 1
    SRV_OK=0
    for attempt in 1 2 3; do
      echo "[judge_demo] starting gz server (attempt $attempt) ..."
      ( unset DISPLAY WAYLAND_DISPLAY; gz sim -s -r "$(pwd)/sim/worlds/tunnel.sdf"           > /tmp/aether_gz_server.log 2>&1 ) &
      GZ_PID=$!
      for i in $(seq 1 30); do
        sleep 1
        if gz topic -l 2>/dev/null | grep -q "/camera/left/image_raw"; then
          if timeout 6 gz topic -e -t /camera/left/image_raw -n 1 2>/dev/null | grep -q stamp; then
            SRV_OK=1; break
          fi
        fi
        kill -0 $GZ_PID 2>/dev/null || break     # server died — retry
      done
      [ "$SRV_OK" = "1" ] && break
      echo "[judge_demo] server not rendering — retrying"
      pkill -9 -f "gz sim" 2>/dev/null; sleep 2
    done
    if [ "$SRV_OK" != "1" ]; then
      echo "[judge_demo] Gazebo sensor rendering failed 3x — falling back to the"
      echo "             guaranteed rig:   ./scripts/judge_demo.sh replay"
      exit 1
    fi
    echo "[judge_demo] gz server rendering ✓ — starting the viewer + ROS stack"
    if [ -n "${DISPLAY:-}" ]; then ( gz sim -g > /tmp/aether_gz_gui.log 2>&1 & ); fi
    trap 'pkill -9 -f "gz sim" 2>/dev/null' EXIT
    exec ros2 launch aether_bringup live3d_stack.launch.py
    ;;
  replay)
    build_ws
    cat <<'TXT'
┌────────────────────────────────────────────────────────────────────┐
│ GUARANTEED RIG — same integrity stack, no Gazebo needed            │
│  · MISSION CONTROL window: click the fault buttons                 │
│  · optional RViz:  rviz2 -d src/aether_bringup/config/aether.rviz  │
└────────────────────────────────────────────────────────────────────┘
TXT
    exec ros2 launch aether_bringup replay.launch.py hud:=true
    ;;
  tour)
    build_ws
    exec bash scripts/demo_tour.sh
    ;;
  chain)
    cat <<'TXT'
┌────────────────────────────────────────────────────────────────────┐
│ REAL OpenVINS stereo-MSCKF on OUR recorded Gazebo tunnel flight    │
│ (pinned ros:jazzy container; deterministic). Takes ~6 minutes.     │
│ Headline: 0.219 % terminal drift over 202.2 m  (DP7 gate < 1.5 %)  │
└────────────────────────────────────────────────────────────────────┘
TXT
    exec docker compose run --rm chain
    ;;
  proof)
    echo "════════ MEASURED RESULTS (provenance: results/) ════════"
    cat results/drift_report.txt
    echo
    echo "Integrity on REAL OpenVINS output: see results/integrity_chain.csv"
    echo "Live-rig measured figures:        docs/figures/measured_*.png"
    echo "Hero animation (synthetic rig):   docs/figures/aether_demo.gif"
    echo "Real-data replay animation:       docs/figures/chain_replay.gif"
    command -v xdg-open > /dev/null && xdg-open docs/figures 2>/dev/null || true
    ;;
  *)
    grep -E '^#( |$)' "$0" | sed 's/^# \{0,1\}//'
    ;;
esac
