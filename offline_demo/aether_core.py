"""
A.E.T.H.E.R core integrity math  —  pure Python, no ROS2 dependency.

  Assured Estimation with Trust, Health & Error-bounded Reckoning.

This module is the single source of truth for the navigation-integrity layer:
  * an oriented eigen-ellipse -> Protection-Level (the "breathing" bound;
    HPL = k * sqrt(lambda_max) of the horizontal covariance block),
  * a dual bound (operational k_op and conservative DAL-C k_ffd),
  * a covariance-conditioned observability proxy D in [0, 1],
  * a NEES consistency check (proves the bound actually covers the truth),
  * constants for the IMU-vs-VIO solution-separation fault detector,
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
# Nominal horizontal 1-sigma (m) used by the observability proxy (slightly above
# the [0.06, 0.06] nominal so that healthy tracking saturates D at 1).
SIGMA_NOM_H: float = 0.08

# --------------------------------------------------------------------------
# Solution-separation (IMU dead-reckoning vs VIO) consistency-fault detector
# --------------------------------------------------------------------------
GRAVITY: float = 9.8           # m/s^2, world frame (accel_world = a_meas - [0,0,g])
SEP_WINDOW_S: float = 2.0      # dead-reckoning channel re-anchors to VIO this often
SEP_NORM_THRESH: float = 3.0   # normalized-separation threshold (in sigma)
SEP_PERSIST_S: float = 0.3     # exceedance must persist this long before flagging
SEP_SIGMA_FLOOR: float = 0.05  # floor (m) on sqrt(lambda_max) in the normalization
SEP_TRUST_CAP: float = 0.3     # trust ceiling while the separation fault is active

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


def horizontal_ellipse(p_pos: np.ndarray, k: float = K_OP) -> tuple[float, float, float]:
    """Oriented horizontal error ellipse from the 2x2 horizontal covariance block.

    Eigendecomposition of P[0:2, 0:2] gives the principal axes of the error
    distribution; the k-sigma ellipse is the bound actually drawn in RViz.

    Returns ``(semi_major, semi_minor, yaw)``: semi-axes in metres and the yaw
    (radians, odom frame) of the major principal axis. Yaw is defined modulo pi
    (an eigenvector and its negation span the same axis).
    """
    H = np.asarray(p_pos, dtype=float)[0:2, 0:2]
    H = 0.5 * (H + H.T)                      # symmetrize against numerical noise
    w, v = np.linalg.eigh(H)                 # ascending eigenvalues, orthonormal vectors
    w = np.clip(w, 1e-12, None)
    major = v[:, 1]                          # eigenvector of lambda_max
    yaw = float(np.arctan2(major[1], major[0]))
    return float(k * np.sqrt(w[1])), float(k * np.sqrt(w[0])), yaw


def horizontal_pl(p_pos: np.ndarray, k: float = K_OP) -> float:
    """Scalar 2-D horizontal protection level = k * sqrt(lambda_max) of the
    horizontal covariance block — the semi-major axis of the oriented ellipse,
    i.e. the bound in the worst-constrained horizontal direction."""
    semi_major, _, _ = horizontal_ellipse(p_pos, k)
    return semi_major


def observability_index(p_pos: np.ndarray) -> float:
    """Covariance-conditioned observability proxy D in [0, 1].

    D = clip(SIGMA_NOM_H^2 / lambda_max(P_2x2), 0, 1) — the constraint ratio of
    the worst horizontal direction vs nominal; the whitened information-matrix
    eigendecomposition remains the documented stretch goal.
    """
    H = np.asarray(p_pos, dtype=float)[0:2, 0:2]
    H = 0.5 * (H + H.T)
    lam_max = float(np.max(np.linalg.eigvalsh(H)))
    return float(np.clip(SIGMA_NOM_H ** 2 / max(lam_max, 1e-12), 0.0, 1.0))


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
    a, b, yaw = horizontal_ellipse(P)
    print("K_OP   =", round(K_OP, 4))
    print("K_DALC =", round(K_DALC, 4))
    print("PL(op) =", protection_level(P).round(4), "m")
    print("HPL    =", round(horizontal_pl(P), 4), "m")
    print("ellipse=", round(a, 4), round(b, 4), "yaw", round(yaw, 4))
    print("D      =", round(observability_index(P), 4))
    print("NEES gate (95%, d=3) =", tuple(round(x, 3) for x in nees_gate()))
