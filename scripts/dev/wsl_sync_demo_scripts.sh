#!/usr/bin/env bash
# Sync the demo-facing scripts into /root/aether (strip CRLF), syntax-check.
set -e
for f in judge_demo.sh demo_tour.sh; do
  tr -d '\r' < "/mnt/d/Work/AETHER/scripts/$f" > "/root/aether/scripts/$f"
  chmod +x "/root/aether/scripts/$f"
  bash -n "/root/aether/scripts/$f"
done
echo SYNCED_OK
