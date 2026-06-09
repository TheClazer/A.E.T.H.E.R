#!/usr/bin/env python3
"""
A.E.T.H.E.R measured-report figures (U12) — CSV in, honest PNGs out.

Reads the CSV written by eval/record_metrics.py (one row per
/nav/integrity_bound message from a live ROS2 run) and renders the four
measured figures to docs/figures/, in the same drafting style as the
offline demo (offline_demo/run_demo.py):

    measured_coverage.png        true error vs dual protection level
    measured_nees.png            filter consistency vs the chi-square gate
    measured_trust_timeline.png  trust + degradation state over the fault beat
    measured_separation.png      solution separation (aided vs IMU-only)

Usage:  python3 eval/make_measured_figures.py [results/metrics.csv]
No ROS2 required — just numpy + matplotlib.
"""
from __future__ import annotations
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# scipy is optional at runtime (same fallback pattern as offline_demo/aether_core.py):
# if unavailable, use the audited chi2.interval(0.95, 3) values.
try:
    from scipy.stats import chi2
    NEES_LO, NEES_HI = (float(v) for v in chi2.interval(0.95, 3))
except Exception:  # pragma: no cover
    NEES_LO, NEES_HI = 0.216, 9.348

FIGDIR = os.path.join(os.path.dirname(__file__), '..', 'docs', 'figures')
INK, STEEL, RED, GOOD, AMBER = "#1B2A36", "#2E5F84", "#C42A1C", "#2F7D4F", "#C98A12"
SUFFIX = " (measured, ROS2 sim)"


def load(csv_path):
    """CSV -> dict of numpy arrays (+ list of state strings)."""
    rows = []
    with open(csv_path, newline='') as fh:
        for row in csv.DictReader(fh):
            rows.append(row)
    if len(rows) < 10:
        sys.exit('make_measured_figures: only %d rows in %s — run '
                 'scripts/run_measured_report.sh first' % (len(rows), csv_path))

    def col(name):
        return np.array([float(r[name]) for r in rows])

    d = {k: col(k) for k in ('t', 'herr', 'hpl_op', 'hpl_dalc', 'ellipse_yaw',
                             'trust', 'nees', 'sep', 'latency', 'fault')}
    d['state'] = [r['state'] for r in rows]
    d['t'] = d['t'] - d['t'][0]          # start the clock at the first bound
    return d


def fault_spans(t, fault):
    """Contiguous fault-active windows as (t_start, t_end) pairs."""
    spans, start = [], None
    for i in range(len(t)):
        if fault[i] > 0.5 and start is None:
            start = t[i]
        elif fault[i] <= 0.5 and start is not None:
            spans.append((start, t[i]))
            start = None
    if start is not None:
        spans.append((start, t[-1]))
    return spans


def detection_latency(d):
    """Prefer the monitor's own /nav/detection_latency report; otherwise derive
    it from the first fault rising edge to the first non-NOMINAL state."""
    lat = d['latency']
    reported = lat[np.isfinite(lat) & (lat > 0.0)]
    if reported.size:
        return float(reported.max())
    spans = fault_spans(d['t'], d['fault'])
    if not spans:
        return float('nan')
    t0 = spans[0][0]
    for i in range(len(d['t'])):
        if d['t'][i] >= t0 and d['state'][i] not in ('NOMINAL', ''):
            return float(d['t'][i] - t0)
    return float('nan')


def shade_faults(ax, spans):
    for a, b in spans:
        ax.axvspan(a, b, color=RED, alpha=0.06)


def run(csv_path):
    os.makedirs(FIGDIR, exist_ok=True)
    d = load(csv_path)
    t = d['t']
    spans = fault_spans(t, d['fault'])
    ok = np.isfinite(d['herr']) & np.isfinite(d['hpl_op'])

    cover_op = float(np.mean(d['herr'][ok] <= d['hpl_op'][ok]) * 100.0) if ok.any() else float('nan')
    latency = detection_latency(d)
    max_pl = float(np.nanmax(d['hpl_op']))
    nofault = (d['fault'] <= 0.5) & np.isfinite(d['nees'])
    anees = float(np.mean(d['nees'][nofault])) if nofault.any() else float('nan')

    # ---- figure 1: integrity-bound coverage (dual bound) ----
    fig, ax = plt.subplots(figsize=(7, 3.2))
    ax.plot(t, d['herr'], color=RED, lw=1.6, label="true horizontal error")
    ax.plot(t, d['hpl_op'], color=STEEL, lw=1.4, label="PL operational (k=2.45)")
    ax.plot(t, d['hpl_dalc'], color=STEEL, lw=1.0, ls="--", label="PL DAL-C (k=6.74)")
    ax.fill_between(t, d['herr'], d['hpl_op'], where=(d['hpl_op'] >= d['herr']),
                    color=GOOD, alpha=0.10)
    shade_faults(ax, spans)
    ax.set_title(f"Integrity-bound coverage — {cover_op:.1f}% inside operational bound{SUFFIX}",
                 fontsize=10)
    ax.set_xlabel("time (s)"); ax.set_ylabel("metres"); ax.legend(fontsize=8)
    ax.grid(True, color="#D7DDE2", lw=0.5); fig.tight_layout()
    fig.savefig(os.path.join(FIGDIR, "measured_coverage.png"), dpi=140); plt.close(fig)

    # ---- figure 2: NEES consistency band ----
    fig, ax = plt.subplots(figsize=(7, 3.2))
    ax.plot(t, d['nees'], color=STEEL, lw=1.0)
    ax.axhspan(NEES_LO, NEES_HI, color=GOOD, alpha=0.12,
               label=f"χ²₃ 95% gate [{NEES_LO:.2f},{NEES_HI:.2f}]")
    ax.axhline(3.0, color=INK, lw=0.8, ls="--", label="E[NEES]=3")
    shade_faults(ax, spans)
    ax.set_title(f"NEES — filter consistency{SUFFIX}", fontsize=10)
    ax.set_xlabel("time (s)"); ax.set_ylabel("NEES")
    finite_nees = d['nees'][np.isfinite(d['nees'])]
    top = np.percentile(finite_nees, 99) if finite_nees.size else 12.0
    ax.set_ylim(0, max(12, top))
    ax.legend(fontsize=8); ax.grid(True, color="#D7DDE2", lw=0.5); fig.tight_layout()
    fig.savefig(os.path.join(FIGDIR, "measured_nees.png"), dpi=140); plt.close(fig)

    # ---- figure 3: trust + state timeline ----
    fig, ax = plt.subplots(figsize=(7, 3.2))
    ax.plot(t, d['trust'], color=STEEL, lw=1.8)
    ax.axhspan(0.8, 1.0, color=GOOD, alpha=0.08); ax.axhspan(0.4, 0.8, color=AMBER, alpha=0.08)
    ax.axhspan(0.0, 0.4, color=RED, alpha=0.08)
    shade_faults(ax, spans)
    inertial_i = next((i for i in range(len(t)) if d['state'][i] == 'INERTIAL'), None)
    if inertial_i is not None and not math.isnan(latency):
        ax.axvline(t[inertial_i], color=RED, lw=1, ls=":")
        ax.annotate(f"INERTIAL +{latency:.2f}s", (t[inertial_i], 0.5), fontsize=8,
                    color=RED, family="monospace")
    ax.set_title(f"Trust score & degradation state{SUFFIX}", fontsize=10)
    ax.set_xlabel("time (s)"); ax.set_ylabel("trust 0–1"); ax.set_ylim(0, 1.02)
    ax.grid(True, color="#D7DDE2", lw=0.5); fig.tight_layout()
    fig.savefig(os.path.join(FIGDIR, "measured_trust_timeline.png"), dpi=140); plt.close(fig)

    # ---- figure 4: solution separation ----
    fig, ax = plt.subplots(figsize=(7, 3.2))
    ax.plot(t, d['sep'], color=STEEL, lw=1.6, label="aided vs IMU-only separation")
    shade_faults(ax, spans)
    ax.set_title(f"Solution separation — RAIM-style cross-check{SUFFIX}", fontsize=10)
    ax.set_xlabel("time (s)"); ax.set_ylabel("metres"); ax.legend(fontsize=8)
    ax.grid(True, color="#D7DDE2", lw=0.5); fig.tight_layout()
    fig.savefig(os.path.join(FIGDIR, "measured_separation.png"), dpi=140); plt.close(fig)

    # ---- summary block ----
    print("=" * 64)
    print("A.E.T.H.E.R MEASURED REPORT — live ROS2 run")
    print("=" * 64)
    print(f"  IB coverage @ k_op=2.45 (%)              {cover_op:.1f}")
    print(f"  fault-detection latency (s)              {latency:.3f}")
    print(f"  max horizontal PL (m)                    {max_pl:.2f}")
    print(f"  ANEES (no-fault window, target~3)        {anees:.2f}")
    print("-" * 64)
    print(f"  figures written to: {os.path.abspath(FIGDIR)}")


if __name__ == '__main__':
    run(sys.argv[1] if len(sys.argv) > 1 else os.path.join('results', 'metrics.csv'))
