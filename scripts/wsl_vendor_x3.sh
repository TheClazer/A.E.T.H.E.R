#!/usr/bin/env bash
# Vendor the Gazebo Fuel "X3 UAV" model (OpenRobotics, Fuel license — see
# model.config) into the repo as sim/models/x3_aether, so the demo never depends
# on a network download at sim start.
set -e
SRC="/home/rayyan/.gz/fuel/fuel.gazebosim.org/openrobotics/models/x3 uav/6"
DST="/mnt/d/Work/AETHER/sim/models/x3_aether"
[ -d "$SRC" ] || { echo "X3 not downloaded at: $SRC"; exit 1; }
rm -rf "$DST"
mkdir -p "$DST"
cp -r "$SRC"/. "$DST"/
echo "=== vendored ==="
ls "$DST"
echo "=== structure ==="
grep -nE "<link name|<joint name|mass>|<model name" "$DST/model.sdf" | head -30
