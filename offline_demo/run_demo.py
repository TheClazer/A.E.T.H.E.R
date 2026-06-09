"""
A.E.T.H.E.R offline demo  —  runs the REAL integrity core over a synthetic
camera-kill flight and produces the report figures + headline metrics.

Run on any machine (no ROS2):   python offline_demo/run_demo.py
Writes PNGs to docs/figures/ and prints the metrics that the live ROS2 demo
will reproduce on the 12-Jun stage.
"""
from __future__ import annotations
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

import aether_core as core
import simulate

FIGDIR = os.path.join(os.path.dirname(__file__), "..", "docs", "figures")
INK, STEEL, RED, GOOD, AMBER = "#1B2A36", "#2E5F84", "#C42A1C", "#2F7D4F", "#C98A12"


def run():
    os.makedirs(FIGDIR, exist_ok=True)
    r = simulate.generate()
    t, x_gt, x_est, P_pos, n_feat = r["t"], r["x_gt"], r["x_est"], r["P_pos"], r["n_feat"]
    n = len(t)

    sm = core.DegradationStateMachine()
    trust = np.zeros(n); state = []; nees = np.zeros(n)
    hpl_op = np.zeros(n); hpl_dalc = np.zeros(n); herr = np.zeros(n)
    tr_prev = None
    for i in range(n):
        P = P_pos[i]
        tr = float(np.trace(P))
        tr_rate = 0.0 if tr_prev is None else (tr - tr_prev) / (1.0 / r["rate_hz"])
        tr_prev = tr
        trust[i] = core.trust_score(n_feat[i], tr)
        state.append(sm.update(n_feat[i], tr_rate, t[i]))
        e = x_est[i] - x_gt[i]
        nees[i] = core.nees(e, P)
        hpl_op[i] = core.horizontal_pl(P, core.K_OP)
        hpl_dalc[i] = core.horizontal_pl(P, core.K_DALC)
        herr[i] = float(np.hypot(e[0], e[1]))

    # ---- headline metrics ----
    cover_op = float(np.mean(herr <= hpl_op) * 100.0)
    cover_dalc = float(np.mean(herr <= hpl_dalc) * 100.0)
    # detection latency: time from outage start to first INERTIAL
    out0 = r["outage"][0]
    inertial_idx = next((i for i in range(n) if t[i] >= out0 and state[i] == "INERTIAL"), None)
    latency = (t[inertial_idx] - out0) if inertial_idx is not None else float("nan")
    # NEES consistency over nominal window
    lo, hi = core.nees_gate()
    nominal_mask = (t < out0)
    anees = float(np.mean(nees[nominal_mask]))
    # drift% vs distance (KITTI-style, using accumulated horizontal error / distance)
    dist = r["speed"] * t
    with np.errstate(divide="ignore", invalid="ignore"):
        drift_pct = np.where(dist > 1.0, herr / dist * 100.0, np.nan)

    metrics = {
        "IB coverage @ k_op=2.45 (%)": round(cover_op, 1),
        "IB coverage @ k_dalc=6.74 (%)": round(cover_dalc, 1),
        "fault-detection latency (s)": round(latency, 3),
        "ANEES (nominal, target~3)": round(anees, 2),
        "NEES 95% gate (d=3)": (round(lo, 2), round(hi, 2)),
        "max horizontal PL during outage (m)": round(float(hpl_op.max()), 2),
        "peak true horizontal error (m)": round(float(herr.max()), 2),
    }

    # ---- figure 1: breathing ellipse over the moving truth ----
    fig, ax = plt.subplots(figsize=(7, 4.2))
    ax.plot(x_est[:, 0], x_est[:, 1], color=STEEL, lw=1.8, label="VIO estimate")
    ax.plot(x_gt[:, 0], x_gt[:, 1], color=INK, lw=1.0, ls="--", label="ground truth")
    for ti in (9.0, 18.0, 28.0):
        i = int(ti * r["rate_hz"])
        col = RED if state[i] == "INERTIAL" else STEEL
        e = Ellipse((x_est[i, 0], x_est[i, 1]), 2 * hpl_op[i], 2 * hpl_op[i] * 0.7,
                    fill=False, edgecolor=col, lw=1.6)
        ax.add_patch(e)
        ax.plot(x_gt[i, 0], x_gt[i, 1], "o", color=RED, ms=4)
        ax.annotate(f"t={ti:.0f}s  PL={hpl_op[i]:.2f}m", (x_est[i, 0], x_est[i, 1] + 0.9),
                    fontsize=7, color=col, ha="center", family="monospace")
    ax.set_title("Breathing integrity bound chasing the moving truth (camera-kill 8–28 s)", fontsize=10)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.legend(fontsize=8, loc="upper left")
    ax.grid(True, color="#D7DDE2", lw=0.5); fig.tight_layout()
    fig.savefig(os.path.join(FIGDIR, "breathing_ellipse.png"), dpi=140); plt.close(fig)

    # ---- figure 2: trust + state timeline ----
    fig, ax = plt.subplots(figsize=(7, 3.2))
    ax.plot(t, trust, color=STEEL, lw=1.8)
    ax.axhspan(0.8, 1.0, color=GOOD, alpha=0.08); ax.axhspan(0.4, 0.8, color=AMBER, alpha=0.08)
    ax.axhspan(0.0, 0.4, color=RED, alpha=0.08)
    ax.axvspan(*r["outage"], color=RED, alpha=0.06)
    if not np.isnan(latency):
        ax.axvline(t[inertial_idx], color=RED, lw=1, ls=":")
        ax.annotate(f"INERTIAL +{latency:.2f}s", (t[inertial_idx], 0.5), fontsize=8,
                    color=RED, family="monospace")
    ax.set_title("Trust score & degradation state", fontsize=10)
    ax.set_xlabel("time (s)"); ax.set_ylabel("trust 0–1"); ax.set_ylim(0, 1.02)
    ax.grid(True, color="#D7DDE2", lw=0.5); fig.tight_layout()
    fig.savefig(os.path.join(FIGDIR, "trust_timeline.png"), dpi=140); plt.close(fig)

    # ---- figure 3: NEES consistency band ----
    fig, ax = plt.subplots(figsize=(7, 3.2))
    ax.plot(t, nees, color=STEEL, lw=1.0)
    ax.axhspan(lo, hi, color=GOOD, alpha=0.12, label=f"χ²₃ 95% gate [{lo:.2f},{hi:.2f}]")
    ax.axhline(3.0, color=INK, lw=0.8, ls="--", label="E[NEES]=3")
    ax.set_title("NEES — filter consistency (the bound is honest)", fontsize=10)
    ax.set_xlabel("time (s)"); ax.set_ylabel("NEES"); ax.set_ylim(0, max(12, np.percentile(nees, 99)))
    ax.legend(fontsize=8); ax.grid(True, color="#D7DDE2", lw=0.5); fig.tight_layout()
    fig.savefig(os.path.join(FIGDIR, "nees_band.png"), dpi=140); plt.close(fig)

    # ---- figure 4: IB coverage (dual bound) ----
    fig, ax = plt.subplots(figsize=(7, 3.2))
    ax.plot(t, herr, color=RED, lw=1.6, label="true horizontal error")
    ax.plot(t, hpl_op, color=STEEL, lw=1.4, label="PL operational (k=2.45)")
    ax.plot(t, hpl_dalc, color=STEEL, lw=1.0, ls="--", label="PL DAL-C (k=6.74)")
    ax.fill_between(t, herr, hpl_op, where=(hpl_op >= herr), color=GOOD, alpha=0.10)
    ax.set_title(f"Integrity-bound coverage — {cover_op:.1f}% inside operational bound", fontsize=10)
    ax.set_xlabel("time (s)"); ax.set_ylabel("metres"); ax.legend(fontsize=8)
    ax.grid(True, color="#D7DDE2", lw=0.5); fig.tight_layout()
    fig.savefig(os.path.join(FIGDIR, "ib_coverage.png"), dpi=140); plt.close(fig)

    # ---- figure 5: drift % vs distance ----
    fig, ax = plt.subplots(figsize=(7, 3.2))
    ax.plot(dist, drift_pct, color=STEEL, lw=1.6)
    ax.axhline(1.5, color=RED, lw=1.5, label="1.5% / 200 m limit")
    ax.fill_between(dist, 0, 1.5, color=GOOD, alpha=0.08)
    ax.set_title("Translational drift vs distance (illustrative / target)", fontsize=10)
    ax.set_xlabel("distance (m)"); ax.set_ylabel("drift %"); ax.set_ylim(0, 3)
    ax.legend(fontsize=8); ax.grid(True, color="#D7DDE2", lw=0.5); fig.tight_layout()
    fig.savefig(os.path.join(FIGDIR, "drift_curve.png"), dpi=140); plt.close(fig)

    print("=" * 64)
    print("A.E.T.H.E.R OFFLINE INTEGRITY DEMO — HEADLINE METRICS")
    print("=" * 64)
    for k, v in metrics.items():
        print(f"  {k:<40} {v}")
    print("-" * 64)
    print(f"  figures written to: {os.path.abspath(FIGDIR)}")
    print("  (ILLUSTRATIVE / TARGET — synthetic rig; real numbers come from the Ubuntu build)")
    return metrics


if __name__ == "__main__":
    run()
