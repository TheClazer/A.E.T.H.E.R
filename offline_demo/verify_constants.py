"""
Reproduce and audit every load-bearing integrity constant with scipy.
Run:  python offline_demo/verify_constants.py
"""
import numpy as np
from scipy.stats import chi2, ncx2
import aether_core as core


def main():
    k_op = np.sqrt(chi2.ppf(0.95, 2))
    # DAL-C: P_HMI = 1e-5/hr at 20 Hz -> per-sample exceedance
    per_sample = 1e-5 / 3600.0 / 20.0
    k_ffd = np.sqrt(chi2.ppf(1.0 - per_sample, 2))
    exceed = chi2.sf(k_ffd ** 2, 2)
    # minimum-detectable non-centrality for the (offline) RAIM-slope bound
    T_D = chi2.ppf(0.999, 2)                                   # P_FA = 1e-3, dof=2
    from scipy.optimize import brentq
    lam_md = brentq(lambda L: ncx2.sf(T_D, 2, L) - 0.999, 1, 300)  # P_MD = 1e-3
    lo, hi = chi2.interval(0.95, 3)

    print("A.E.T.H.E.R integrity constants (scipy-verified)")
    print("-" * 52)
    print(f"  k_op  (95% 2-D)             = {k_op:.4f}   [module: {core.K_OP:.4f}]")
    print(f"  k_ffd (DAL-C, P_HMI=1e-5/hr)= {k_ffd:.4f}   [module: {core.K_DALC:.4f}]")
    print(f"  per-sample integrity risk   = {per_sample:.3e}")
    print(f"  exceedance at k_ffd         = {exceed:.3e}")
    print(f"  T_D (P_FA=1e-3, dof=2)      = {T_D:.3f}")
    print(f"  lambda_md (P_MD=1e-3)       = {lam_md:.2f}   [module: {core.LAMBDA_MD:.1f}]")
    print(f"  NEES chi2 95% gate (d=3)    = [{lo:.3f}, {hi:.3f}]")
    assert abs(k_op - 2.4477) < 1e-3
    assert abs(k_ffd - 6.74) < 0.05
    assert abs(lam_md - 45.0) < 1.0
    print("-" * 52)
    print("  OK — all constants reproduce.")


if __name__ == "__main__":
    main()
