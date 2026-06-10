# A.E.T.H.E.R — Judge Briefing (one page)

**Team:** Rayyan & Ashitha · **Problem:** DP7 — Autonomous Navigator for GPS-Denied Environments
**One sentence:** an integrity-aware Visual-Inertial Odometry stack — it doesn't just localize a GPS-denied drone, it publishes a live, statistically honest **bound on its own error** and **knows within half a second when it can no longer be trusted** (the GNSS-RAIM idea, ported to the vision aid).

## The three measured results

| # | Claim | Number | Where the evidence lives |
|---|---|---|---|
| 1 | Real VIO accuracy on **our own** simulated mission (not a benchmark dataset): OpenVINS stereo-MSCKF on the 202.2 m Gazebo tunnel flight | **0.219 % terminal drift** (DP7 gate < 1.5 % — 6.8× margin), RMS ATE 0.202 m | `results/drift_report.txt`; rerun live: `docker compose run --rm chain` (~6 min, deterministic) |
| 2 | The integrity bound is **honest on the real VIO**: the same monitor nodes ran live against the OpenVINS output | bound covered the true error in **97.2 %** of 31,173 verdicts | `results/integrity_chain.csv` |
| 3 | The integrity rig measured live in ROS2 under injected faults | coverage **97.6 %** (emergent), **ANEES 3.05** (χ²₃ expectation ≈ 3 ⇒ the filter is candid about its own uncertainty), detection **1.18 s** | `results/metrics.csv`, `docs/figures/measured_*.png` |

## What to ask the team to run (pick any)

1. **`./scripts/judge_demo.sh live3d`** — the showpiece. A physics-real X3 quad patrols a 200 m textured tunnel in Gazebo. RViz shows the truth trail vs the estimate trail, the **breathing protection-level ellipse**, the naive ghost, and the **live camera pane**. The **Mission Control** window has clickable buttons:
   - **KILL CAMERA** → the camera pane actually goes dark (the image stream stops), trust collapses, INERTIAL in <1 s, the bound blooms *and keeps the truth inside* — while the naive ghost (a system with no integrity layer) stays confidently small and **loses the truth**.
   - **IMU BIAS** → the camera looks perfectly healthy; the **solution-separation** cross-check (an independent IMU dead-reckoning channel — the ARAIM concept) catches it anyway.
   - **FEATURE STARVATION** → graceful DEGRADED, not panic.
   - **UWB AID** → a layered aiding modality clamps the error mid-blackout (the Honeywell-HANA pattern).
2. **`./scripts/judge_demo.sh tour`** — the same story, scripted and narrated, hands-free (~2.5 min).
3. **`./scripts/judge_demo.sh chain`** — watch the *real* OpenVINS number regenerate from the committed pipeline.

## How the evidence is layered (our honesty policy)

- **Real-VIO measurements** (rows 1–2 above): recorded, deterministic, reproducible — real rendered stereo images, real physics IMU, real MSCKF.
- **Live rig measurements** (row 3): real ROS2, seeded VIO-class error model, statistics emergent (never fitted to the bound).
- **The live 3D scene**: the same validated error model riding the live simulator — labeled on-screen, because upstream OpenVINS does not build on Ubuntu 26.04 yet (CMake 4 / Boost 1.90 / removed `ament_target_dependencies`); that is exactly why the real-VIO chain ships pinned in a `ros:jazzy` container.

## Why it matters (the Honeywell frame)

Accuracy alone is table stakes; **certifiable navigation needs integrity** — a bound you can trust and a system that degrades gracefully instead of lying confidently. A.E.T.H.E.R implements that layer end-to-end at student scale: dual protection level (operational `k=2.45` / DAL-C `k_ffd=6.74`, derived from a stated integrity-risk allocation, scipy-reproducible), NEES-proven candor, solution-separation fault detection, characterized inertial fallback (HG4930-class, physics-cross-checked ±20 %), and layered aiding — the same philosophy as Assured PNT / HANA.

## Repo map (60 seconds)

`scripts/judge_demo.sh` (start here) · `offline_demo/` (the math, runs anywhere, tested) · `src/` (ROS2 nodes incl. Mission Control) · `sim/` (tunnel + X3 drone) · `eval/` (metrics tooling) · `results/` (the evidence) · `docs/` (bible, manual, runbook, deck) · CI: lint + tests + full `ros:jazzy` workspace build on every push.
