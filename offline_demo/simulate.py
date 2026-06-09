"""
Synthetic VIO run generator for the A.E.T.H.E.R offline demo.

Produces a deterministic, physically-plausible 32 s flight down a tunnel at
1.5 m/s with a scripted CAMERA-KILL outage on [8 s, 28 s]:

  * NOMINAL  : ~120 tracked features, tight position covariance.
  * OUTAGE   : features collapse to ~0; the filter dead-reckons, so the position
               covariance blooms (sigma grows ~linearly with outage time) and the
               estimate drifts away from ground truth but STAYS inside the bound.
  * RE-ACQUIRE: features return, covariance contracts.

No ROS2 / no Gazebo — this is the controllable rig that lets us validate the
integrity math anywhere. Returns numpy arrays sampled at 20 Hz.
"""
from __future__ import annotations
import numpy as np

RATE_HZ = 20.0
DURATION_S = 32.0
SPEED = 1.5                 # m/s cruise
OUTAGE = (8.0, 28.0)        # camera-kill window
SEED = 7                    # deterministic


def generate():
    rng = np.random.default_rng(SEED)
    n = int(DURATION_S * RATE_HZ)
    t = np.arange(n) / RATE_HZ

    # ground-truth path: gentle S-curve corridor, monotonic in x
    x = SPEED * t
    y = 0.6 * np.sin(2.0 * np.pi * t / 18.0)
    z = 1.5 + 0.05 * np.sin(2.0 * np.pi * t / 7.0)
    x_gt = np.stack([x, y, z], axis=1)

    n_feat = np.full(n, 120.0)
    sigma = np.zeros((n, 3))        # per-axis 1-sigma position uncertainty (m)
    x_est = np.zeros((n, 3))
    drift = np.zeros(3)             # accumulated dead-reckoning drift vector

    sig_nom = np.array([0.06, 0.06, 0.04])     # nominal 1-sigma
    in_outage_prev = False
    for i in range(n):
        ti = t[i]
        in_outage = OUTAGE[0] <= ti < OUTAGE[1]
        if in_outage:
            # features collapse (with a little noise so it crosses thresholds cleanly)
            n_feat[i] = max(0.0, rng.normal(3.0, 2.0))
            dt_out = ti - OUTAGE[0]
            # dead-reckoning: 1-sigma grows ~linearly -> ~1.05 m at 20 s outage
            sigma[i] = sig_nom + np.array([0.05, 0.05, 0.03]) * dt_out
            # the estimate drifts; keep the true error at ~0.8 * sigma (inside the bound)
            if not in_outage_prev:
                drift = np.zeros(3)
            drift = 0.8 * sigma[i] * np.array([1.0, -0.7, 0.4])
            x_est[i] = x_gt[i] + drift
        else:
            n_feat[i] = max(60.0, rng.normal(120.0, 8.0))
            if in_outage_prev:
                # re-acquire: covariance contracts back over ~1.5 s
                pass
            sigma[i] = sig_nom + rng.normal(0.0, 0.004, 3)
            sigma[i] = np.clip(sigma[i], 0.02, None)
            # small consistent estimate error within ~1 sigma
            x_est[i] = x_gt[i] + sigma[i] * rng.normal(0.0, 0.6, 3)
        in_outage_prev = in_outage

    # re-acquire smoothing: blend sigma down for 1.5 s after outage ends
    end_idx = int(OUTAGE[1] * RATE_HZ)
    for k in range(end_idx, min(end_idx + int(1.5 * RATE_HZ), n)):
        a = (k - end_idx) / (1.5 * RATE_HZ)
        sigma[k] = (1 - a) * sigma[end_idx - 1] + a * sig_nom
        x_est[k] = x_gt[k] + sigma[k] * (1 - a) * np.array([0.8, -0.5, 0.3])
        n_feat[k] = 40.0 + a * 80.0

    # build per-step 3x3 covariance and the ROS-style 6x6 odom covariance
    P_pos = np.array([np.diag(s ** 2) for s in sigma])
    odom_cov = np.zeros((n, 36))
    for i in range(n):
        C = np.zeros((6, 6))
        C[0:3, 0:3] = P_pos[i]
        C[3:6, 3:6] = np.diag([1e-4, 1e-4, 1e-4])  # orientation block (unused by proxy)
        odom_cov[i] = C.reshape(-1)

    return {
        "t": t, "x_gt": x_gt, "x_est": x_est, "P_pos": P_pos,
        "odom_cov": odom_cov, "n_feat": n_feat, "outage": OUTAGE,
        "rate_hz": RATE_HZ, "speed": SPEED,
    }


if __name__ == "__main__":
    r = generate()
    print(f"generated {len(r['t'])} samples, {r['t'][-1]:.1f}s, outage {r['outage']}")
    print(f"feature range: {r['n_feat'].min():.0f}..{r['n_feat'].max():.0f}")
