"""
A.E.T.H.E.R core integrity math  —  pure Python, no ROS2 dependency.

  Assured Estimation with Trust, Health & Error-bounded Reckoning.

This module is the single source of truth for the navigation-integrity layer:
  * a covariance-trace -> Protection-Level proxy (the "breathing" bound),
  * a dual bound (operational k_op and conservative DAL-C k_ffd),
  * a NEES consistency check (proves the bound actually covers the truth),
  * a hysteretic, debounced R(D) degradation state machine.

It is imported BOTH by the offline demo (offline_demo/run_demo.py, runs on any
laptop) AND mirrored by the ROS2 node (src/aether_integrity_monitor). Keeping the
math here, framework-free, is what lets us validate it without Ubuntu/ROS2/Gazebo.

The bound is RELATIVE (local/segment) — the GNSS-RAIM analogue applied to the
vision aid, on the segment-RPE quantity DP7 grades. See docs/AETHER_BIBLE.pdf.
"""
from __future__ import annotations
import numpy as np

# scipy is used to *derive* the integrity constants; if it's unavailable at runtime
# (e.g. a very new Python where no scipy wheel exists yet) we fall back to the audited
# values. The ROS2 node therefore needs only numpy. Re-derive: verify_constants.py.
try:
    from scipy.stats import chi2
    _HAVE_SCIPY = True
except Exception:  # pragma: no cover
    chi2 = None
    _HAVE_SCIPY = False

# --------------------------------------------------------------------------
# Audited protection-level k-factors.  Reproduce with offline_demo/verify_constants.py
# --------------------------------------------------------------------------
# Operational bound: 95% containment of a 2-D (horizontal) Gaussian error.
K_OP: float = float(np.sqrt(chi2.ppf(0.95, 2))) if _HAVE_SCIPY else 2.4477
# DAL-C "as-if" bound: DO-178C integrity-risk allocation P_HMI = 1e-5/hr at 20 Hz
# -> per-sample exceedance 1.39e-10 -> a deliberately conservative envelope.
K_DALC: float = float(np.sqrt(chi2.ppf(1.0 - 1.39e-10, 2))) if _HAVE_SCIPY else 6.7374
# Missed-detection non-centrality used in the (offline/stretch) RAIM-slope bound.
LAMBDA_MD: float = 45.0

# --------------------------------------------------------------------------
# Degradation thresholds  (mirror src/aether_bringup/config/integrity.yaml)
# --------------------------------------------------------------------------
N_NOMINAL: float = 120.0     # nominal tracked-feature count
TR_NOMINAL: float = 0.02     # nominal trace(P_pos) in m^2
N_DEGRADED: int = 80         # below this -> DEGRADED
N_INERTIAL: int = 10         # below this (for DEBOUNCE_S) -> INERTIAL
N_REACQUIRE: int = 40        # above this (from INERTIAL) -> RE_ACQUIRE
DEBOUNCE_S: float = 0.3      # time to confirm vision loss ("we know within half a second")


def position_cov(odom_cov_6x6) -> np.ndarray:
    """Extract the 3x3 position covariance from a ROS ``nav_msgs/Odometry``
    ``pose.covariance`` (row-major 6x6, ordered [x, y, z, rx, ry, rz]).

    Position is the TOP-LEFT [0:3, 0:3] block (per the ROS message contract).
    """
    C = np.asarray(odom_cov_6x6, dtype=float).reshape(6, 6)
    return C[0:3, 0:3]


def protection_level(p_pos: np.ndarray, k: float = K_OP) -> np.ndarray:
    """Per-axis protection level = k * 1-sigma. Returns array([pl_x, pl_y, pl_z]) in metres."""
    sigma = np.sqrt(np.clip(np.diag(p_pos), 1e-12, None))
    return k * sigma


def horizontal_pl(p_pos: np.ndarray, k: float = K_OP) -> float:
    """Scalar 2-D horizontal protection level (the radius the breathing ellipse uses)."""
    pl = protection_level(p_pos, k)
    return float(np.hypot(pl[0], pl[1]))


def nees(err_xyz, p_pos: np.ndarray) -> float:
    """Normalized Estimation Error Squared (chi-square, dim=3):  e^T P^-1 e."""
    e = np.asarray(err_xyz, dtype=float)
    return float(e @ np.linalg.inv(p_pos) @ e)


def nees_gate(conf: float = 0.95, dim: int = 3) -> tuple[float, float]:
    """Two-sided chi-square acceptance band for a consistent filter (E[NEES] = dim)."""
    if _HAVE_SCIPY:
        lo, hi = chi2.interval(conf, dim)
        return float(lo), float(hi)
    return (0.216, 9.348)  # chi2.interval(0.95, 3), audited fallback


def trust_score(n_feat: float, trace_pos: float,
                w_feat: float = 0.6, w_cov: float = 0.4) -> float:
    """Continuous 0..1 trust: feature richness + covariance tightness."""
    f = min(n_feat / N_NOMINAL, 1.0)
    g = min(TR_NOMINAL / max(trace_pos, 1e-9), 1.0)
    return float(np.clip(w_feat * f + w_cov * g, 0.0, 1.0))


def trust_color(trust: float) -> str:
    """Green / amber / red for the cockpit HUD."""
    if trust >= 0.8:
        return "GREEN"
    if trust >= 0.4:
        return "AMBER"
    return "RED"


class DegradationStateMachine:
    """Continuous R(D) degradation logic, hysteretic + debounced.

    NOMINAL -> DEGRADED -> INERTIAL -> RE_ACQUIRE -> NOMINAL.
    Enters INERTIAL only after the feature count stays below N_INERTIAL for
    DEBOUNCE_S seconds (kills chatter; this debounce IS the "< 0.5 s" detection).
    """
    NOMINAL = "NOMINAL"
    DEGRADED = "DEGRADED"
    INERTIAL = "INERTIAL"
    RE_ACQUIRE = "RE_ACQUIRE"

    def __init__(self, debounce_s: float = DEBOUNCE_S):
        self.state = self.NOMINAL
        self.debounce_s = debounce_s
        self._t_below = None  # time at which n_feat first dropped below N_INERTIAL

    def update(self, n_feat: float, trace_rate: float, t_now: float) -> str:
        if n_feat < N_INERTIAL:
            if self._t_below is None:
                self._t_below = t_now
            if (t_now - self._t_below) >= self.debounce_s:
                self.state = self.INERTIAL
            elif self.state == self.NOMINAL:
                self.state = self.DEGRADED  # provisional, pre-debounce
        else:
            self._t_below = None
            if self.state == self.INERTIAL:
                if n_feat >= N_REACQUIRE:
                    self.state = self.RE_ACQUIRE
            elif self.state == self.RE_ACQUIRE:
                if n_feat >= N_DEGRADED and trace_rate <= 0.0:
                    self.state = self.NOMINAL
            else:
                self.state = self.NOMINAL if (n_feat >= N_DEGRADED and trace_rate <= 0.0) else self.DEGRADED
        return self.state


if __name__ == "__main__":  # tiny smoke test
    P = np.diag([0.01, 0.01, 0.02])
    print("K_OP   =", round(K_OP, 4))
    print("K_DALC =", round(K_DALC, 4))
    print("PL(op) =", protection_level(P).round(4), "m")
    print("HPL    =", round(horizontal_pl(P), 4), "m")
    print("NEES gate (95%, d=3) =", tuple(round(x, 3) for x in nees_gate()))
