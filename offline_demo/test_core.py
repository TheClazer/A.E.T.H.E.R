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
    assert coverage >= 0.95                    # the bound covers the truth 95%+
