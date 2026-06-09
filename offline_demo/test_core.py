"""
pytest unit tests for the A.E.T.H.E.R integrity core.
Run:  pytest offline_demo/test_core.py -q
"""
import numpy as np
import aether_core as core
import simulate


def test_k_factors():
    assert abs(core.K_OP - 2.4477) < 1e-3
    assert abs(core.K_DALC - 6.74) < 0.05
    assert core.K_DALC > core.K_OP            # DAL-C is the conservative envelope


def test_protection_level_nonneg_and_scales():
    P = np.diag([0.04, 0.09, 0.01])           # sigma 0.2, 0.3, 0.1
    pl = core.protection_level(P, core.K_OP)
    assert np.all(pl >= 0)
    assert np.allclose(pl, core.K_OP * np.array([0.2, 0.3, 0.1]), atol=1e-9)
    # larger covariance -> larger bound (it "breathes")
    assert core.horizontal_pl(P * 4) > core.horizontal_pl(P)
    # HPL routes through the ellipse: k * sqrt(lambda_max) = k * sigma_max
    assert abs(core.horizontal_pl(P, core.K_OP) - core.K_OP * 0.3) < 1e-9


def test_horizontal_ellipse_oriented():
    # synthetic CORRELATED horizontal block: principal axes at 30 degrees,
    # sigma 0.3 along the major axis and 0.1 along the minor axis
    th = np.deg2rad(30.0)
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    P = np.eye(3) * 0.01
    P[0:2, 0:2] = R @ np.diag([0.09, 0.01]) @ R.T
    a, b, yaw = core.horizontal_ellipse(P, core.K_OP)
    assert abs(a - core.K_OP * 0.3) < 1e-9
    assert abs(b - core.K_OP * 0.1) < 1e-9
    # yaw is defined modulo pi (eigenvector sign ambiguity)
    dyaw = (yaw - th + np.pi / 2.0) % np.pi - np.pi / 2.0
    assert abs(dyaw) < 1e-9
    # the scalar HPL is exactly the semi-major axis
    assert abs(core.horizontal_pl(P, core.K_OP) - a) < 1e-12


def test_observability_index_drops_during_outage():
    # nominal covariance -> fully observable (saturates at 1)
    P_nom = np.diag([0.06, 0.06, 0.04]) ** 2
    assert core.observability_index(P_nom) == 1.0
    # blown-up covariance -> a horizontal direction is unconstrained -> D ~ 0
    P_blown = np.diag([1.0, 1.0, 0.5]) ** 2
    assert core.observability_index(P_blown) < 0.01
    # and D is emergent on the synthetic run: high before the outage, ~0 inside it
    r = simulate.generate()
    t = r["t"]
    D = np.array([core.observability_index(P) for P in r["P_pos"]])
    out0, out1 = r["outage"]
    assert np.mean(D[t < out0]) >= 0.95
    assert D[(t > out0 + 5.0) & (t < out1)].max() < 0.05


def test_nees_consistency_band():
    # a sample drawn at exactly 1-sigma on each axis -> NEES = 3
    P = np.diag([0.04, 0.04, 0.04])
    e = np.sqrt(np.diag(P))                    # one sigma per axis
    assert abs(core.nees(e, P) - 3.0) < 1e-6
    lo, hi = core.nees_gate(0.95, 3)
    assert lo < 3.0 < hi


def test_state_machine_detects_and_recovers():
    sm = core.DegradationStateMachine(debounce_s=0.3)
    t = 0.0
    # nominal
    for _ in range(10):
        sm.update(120, 0.0, t); t += 0.05
    assert sm.state == core.DegradationStateMachine.NOMINAL
    # camera dies -> after debounce -> INERTIAL within ~0.3-0.4 s
    entered = None
    for _ in range(20):
        sm.update(0, 1.0, t)
        if sm.state == core.DegradationStateMachine.INERTIAL and entered is None:
            entered = t
        t += 0.05
    assert sm.state == core.DegradationStateMachine.INERTIAL
    assert entered is not None and entered <= 0.5 + 10 * 0.05  # within ~0.5 s of the drop
    # vision returns -> RE_ACQUIRE -> NOMINAL
    for _ in range(20):
        sm.update(120, -0.1, t); t += 0.05
    assert sm.state in (core.DegradationStateMachine.NOMINAL,
                        core.DegradationStateMachine.RE_ACQUIRE)


def test_synthetic_run_coverage():
    r = simulate.generate()
    herr, hpl = [], []
    for i in range(len(r["t"])):
        P = r["P_pos"][i]
        e = r["x_est"][i] - r["x_gt"][i]
        herr.append(np.hypot(e[0], e[1]))
        hpl.append(core.horizontal_pl(P, core.K_OP))
    herr, hpl = np.array(herr), np.array(hpl)
    coverage = np.mean(herr <= hpl)
    # the eigen-ellipse HPL (k*sqrt(lambda_max)) is TIGHTER than the old per-axis
    # hypot proxy, so coverage is emergent, not engineered: require >= 0.90
    assert coverage >= 0.90
