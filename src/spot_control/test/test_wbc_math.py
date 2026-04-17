import numpy as np
import pytest
from spot_control.wbc_math import (
    compute_j_base,
    compute_j_holistic,
    manipulability,
    wbc_split,
)


# ── compute_j_base ────────────────────────────────────────────────────────────

def test_j_base_shape():
    p_ee = np.array([0.5, 0.0, 0.3])
    J = compute_j_base(p_ee)
    assert J.shape == (6, 2)


def test_j_base_pure_vx():
    """Pure forward base motion → EE moves forward, no angular contribution."""
    p_ee = np.array([0.5, 0.0, 0.0])
    J = compute_j_base(p_ee)
    v = J @ np.array([1.0, 0.0])   # vx=1, wz=0
    assert abs(v[3] - 1.0) < 1e-9  # linear x (row 3)
    assert abs(v[4]) < 1e-9        # linear y
    assert abs(v[2]) < 1e-9        # angular z


def test_j_base_pure_wz():
    """Pure yaw → EE has linear velocity from rotation + angular z."""
    p_ee = np.array([1.0, 0.0, 0.0])
    J = compute_j_base(p_ee)
    v = J @ np.array([0.0, 1.0])   # vx=0, wz=1
    # linear y = p_ee_x * wz = 1.0
    assert abs(v[4] - 1.0) < 1e-9
    # angular z = wz = 1.0
    assert abs(v[2] - 1.0) < 1e-9


def test_j_base_lateral_ee():
    """EE offset on y → vx produces no coupling (wz=0)."""
    p_ee = np.array([0.0, 0.5, 0.0])
    J = compute_j_base(p_ee)
    v = J @ np.array([1.0, 0.0])
    assert abs(v[3] - 1.0) < 1e-9   # linear x = vx
    assert abs(v[4]) < 1e-9         # linear y = 0


def test_j_base_wz_coupling_with_y_offset():
    """EE at py=0.5, wz=1 → linear x = -py*wz = -0.5."""
    p_ee = np.array([0.0, 0.5, 0.0])
    J = compute_j_base(p_ee)
    v = J @ np.array([0.0, 1.0])
    assert abs(v[3] - (-0.5)) < 1e-9   # vx_ee = -py * wz


# ── compute_j_holistic ────────────────────────────────────────────────────────

def test_j_holistic_shape():
    J_arm = np.random.randn(6, 6)
    J_base = np.random.randn(6, 2)
    J = compute_j_holistic(J_arm, J_base)
    assert J.shape == (6, 8)


def test_j_holistic_concatenation():
    J_arm = np.eye(6)
    J_base = np.ones((6, 2))
    J = compute_j_holistic(J_arm, J_base)
    np.testing.assert_array_equal(J[:, :6], J_arm)
    np.testing.assert_array_equal(J[:, 6:], J_base)


# ── manipulability ────────────────────────────────────────────────────────────

def test_manipulability_positive():
    J = np.eye(6)
    m = manipulability(J)
    assert m > 0.0


def test_manipulability_singular():
    """Near-singular Jacobian → near-zero manipulability."""
    J = np.zeros((6, 6))
    J[0, 0] = 1.0
    m = manipulability(J)
    assert m < 1e-6


def test_manipulability_identity():
    """Identity Jacobian → manipulability = 1."""
    J = np.eye(6)
    m = manipulability(J)
    assert abs(m - 1.0) < 1e-9


# ── wbc_split ─────────────────────────────────────────────────────────────────

def test_wbc_split_output_shape():
    J_holistic = np.random.randn(6, 8)
    v_des = np.random.randn(6)
    q_dot, vx, wz = wbc_split(J_holistic, v_des, m=0.5)
    assert q_dot.shape == (6,)
    assert np.isscalar(vx)
    assert np.isscalar(wz)


def test_wbc_split_zero_desired():
    """Zero desired velocity → zero output."""
    J_holistic = np.eye(6, 8)
    v_des = np.zeros(6)
    q_dot, vx, wz = wbc_split(J_holistic, v_des, m=0.5)
    np.testing.assert_allclose(q_dot, np.zeros(6), atol=1e-9)
    assert abs(vx) < 1e-9
    assert abs(wz) < 1e-9


def test_wbc_split_high_manipulability_prefers_arm():
    """High m → low arm weight → arm absorbs more of the task."""
    J = np.zeros((6, 8))
    J[0, 0] = 1.0   # arm joint 0 affects EE x
    J[0, 6] = 1.0   # base vx affects EE x
    v_des = np.zeros(6)
    v_des[0] = 1.0

    _, vx_low_m, _ = wbc_split(J, v_des, m=0.01,
                                lam_arm=1.0, lam_base=1.0, damping=1e-3)
    _, vx_high_m, _ = wbc_split(J, v_des, m=1.0,
                                 lam_arm=1.0, lam_base=1.0, damping=1e-3)

    assert abs(vx_low_m) >= abs(vx_high_m)


def test_wbc_split_velocity_limits():
    """Output velocities must respect limits."""
    J_holistic = np.ones((6, 8)) * 0.1
    v_des = np.ones(6) * 100.0
    q_dot, vx, wz = wbc_split(J_holistic, v_des, m=0.5,
                               vx_max=0.4, wz_max=0.5, q_dot_max=0.6)
    assert abs(vx) <= 0.4 + 1e-9
    assert abs(wz) <= 0.5 + 1e-9
    assert np.all(np.abs(q_dot) <= 0.6 + 1e-9)
