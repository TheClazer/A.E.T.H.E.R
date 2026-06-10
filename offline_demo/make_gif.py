"""
Render the README hero GIF: top-down view of the kill-the-camera beat.

AETHER's oriented protection-level ellipse (eigen-PL) blooms through the outage
and keeps the truth inside; the naive fixed-sigma ghost stays confidently small
and loses the truth. Pure offline (simulate.py + aether_core) — same math as
the live stack.

    python offline_demo/make_gif.py     ->  docs/figures/aether_demo.gif
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from matplotlib.animation import FuncAnimation, PillowWriter

import aether_core as core
import simulate

OUT = os.path.join(os.path.dirname(__file__), "..", "docs", "figures", "aether_demo.gif")
INK, STEEL, RED, GOOD, AMBER, MUT = "#1B2A36", "#2E5F84", "#C42A1C", "#2F7D4F", "#C98A12", "#7E8A93"
NAIVE_SIGMA = 0.08          # what a no-integrity system keeps claiming (m, 1-sigma)


def main():
    r = simulate.generate()
    t, x_gt, x_est, P = r["t"], r["x_gt"], r["x_est"], r["P_pos"]
    n = len(t)
    sm = core.DegradationStateMachine()
    tr_prev = None
    states, hpls, yaws, axes_ab = [], [], [], []
    for i in range(n):
        tr = float(np.trace(P[i]))
        tr_rate = 0.0 if tr_prev is None else (tr - tr_prev) * r["rate_hz"]
        tr_prev = tr
        states.append(sm.update(r["n_feat"][i], tr_rate, t[i]))
        a, b, yaw = core.horizontal_ellipse(P[i], core.K_OP)
        axes_ab.append((a, b)); yaws.append(yaw)
        hpls.append(core.horizontal_pl(P[i], core.K_OP))

    step = 4                            # 20 Hz sim -> 5 fps frames
    idxs = list(range(0, n, step))
    fig, ax = plt.subplots(figsize=(8.4, 4.0), dpi=92)
    fig.patch.set_facecolor("#F7F6F2")

    def draw(k):
        i = idxs[k]
        ax.clear()
        ax.set_facecolor("#F7F6F2")
        ax.plot(x_gt[:i + 1, 0], x_gt[:i + 1, 1], color=INK, lw=1.0, ls="--", label="ground truth")
        ax.plot(x_est[:i + 1, 0], x_est[:i + 1, 1], color=STEEL, lw=1.6, label="VIO estimate")
        st = states[i]
        col = {"NOMINAL": GOOD, "DEGRADED": AMBER, "INERTIAL": RED, "RE_ACQUIRE": STEEL}[st]
        a, b = axes_ab[i]
        ax.add_patch(Ellipse((x_est[i, 0], x_est[i, 1]), 2 * a, 2 * b,
                             angle=np.degrees(yaws[i]), fill=False, edgecolor=col, lw=2.2))
        # naive ghost: frozen confidence, never blooms
        npl = core.K_OP * NAIVE_SIGMA
        ax.add_patch(Ellipse((x_est[i, 0], x_est[i, 1]), 2 * npl, 2 * npl,
                             fill=False, edgecolor=MUT, lw=1.2, ls=":"))
        ax.plot(x_gt[i, 0], x_gt[i, 1], "o", color=RED, ms=6)
        err = float(np.hypot(*(x_est[i, :2] - x_gt[i, :2])))
        inside = err <= hpls[i]
        naive_inside = err <= npl
        ax.set_title(
            f"A.E.T.H.E.R  ·  t={t[i]:4.1f}s  ·  {st:<10s}  ·  "
            f"PL {hpls[i]:.2f} m — truth {'INSIDE' if inside else 'OUTSIDE'}   |   "
            f"naive ghost: truth {'inside' if naive_inside else 'LOST'}",
            fontsize=9, color=INK, family="monospace")
        if r["outage"][0] <= t[i] < r["outage"][1]:
            ax.text(0.02, 0.04, "CAMERA DEAD — inertial coast, bound blooming",
                    transform=ax.transAxes, fontsize=9, color=RED, family="monospace")
        ax.set_xlim(-2, 50); ax.set_ylim(-2.4, 2.4)
        ax.set_xlabel("x (m)", fontsize=8); ax.set_ylabel("y (m)", fontsize=8)
        ax.grid(True, color="#D7DDE2", lw=0.5)
        ax.legend(loc="upper left", fontsize=7)

    anim = FuncAnimation(fig, draw, frames=len(idxs), interval=120)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    anim.save(OUT, writer=PillowWriter(fps=9))
    print("wrote", os.path.abspath(OUT), f"({os.path.getsize(OUT)/1e6:.1f} MB, {len(idxs)} frames)")


if __name__ == "__main__":
    main()
