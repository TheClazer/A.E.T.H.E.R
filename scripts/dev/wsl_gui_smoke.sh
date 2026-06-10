#!/usr/bin/env bash
# P0 — WSLg GUI smoke test: can rviz2, the Gazebo GUI client, and an interactive
# matplotlib backend open windows under WSLg? Each is launched briefly; we check
# the process survives past window creation and no display errors appear.
source /opt/ros/lyrical/setup.bash

echo "=== display env ==="
echo "DISPLAY=$DISPLAY  WAYLAND_DISPLAY=$WAYLAND_DISPLAY  XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR"
ls /tmp/.X11-unix/ 2>/dev/null || echo "(no X sockets)"

echo "=== [1/3] matplotlib interactive backend ==="
python3 - <<'EOF'
import importlib
ok = []
for backend, mod in [("TkAgg", "tkinter"), ("QtAgg", "PyQt5")]:
    try:
        importlib.import_module(mod)
        ok.append(backend)
    except Exception:
        pass
print("importable backends:", ok or "NONE")
if ok:
    import matplotlib
    matplotlib.use(ok[0])
    import matplotlib.pyplot as plt
    fig = plt.figure()
    fig.canvas.draw()           # forces window/canvas creation
    print(f"MPL_{ok[0]}_OK")
    plt.close(fig)
else:
    print("MPL_NEEDS_PACKAGE")
EOF

echo "=== [2/3] rviz2 (8 s) ==="
timeout 8 rviz2 > /tmp/rviz_smoke.log 2>&1
RC=$?
# timeout(124) = it RAN for 8 s (good). quick non-zero exit = crashed.
if [ "$RC" = "124" ]; then echo "RVIZ_OK (ran 8 s)"; else echo "RVIZ_RC=$RC"; tail -5 /tmp/rviz_smoke.log; fi

echo "=== [3/3] gz sim GUI client + server (12 s, empty world) ==="
timeout 12 gz sim -r ~/aether/sim/worlds/tunnel.sdf > /tmp/gz_gui_smoke.log 2>&1
RC=$?
if [ "$RC" = "124" ]; then echo "GZ_GUI_OK (ran 12 s)"; else echo "GZ_GUI_RC=$RC"; grep -iE "error|fail|display" /tmp/gz_gui_smoke.log | head -5; fi
pkill -9 -f "gz sim" 2>/dev/null
echo "GUI_SMOKE_DONE"
