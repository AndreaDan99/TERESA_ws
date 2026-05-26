"""
wbc_math.py — Pure WBC math functions, no ROS dependencies.

Arm-only WBC for look-at task:
  damped_pinv(J)          → damped pseudo-inverse J⁺
  null_space_projector(J) → N = I − J⁺J  (null-space projection)
  manipulability(J_arm)   → Yoshikawa measure m = √det(JJᵀ)

Pinocchio spatial velocity convention: [omega(3), v(3)]
  rows 0-2 = angular  (wx, wy, wz)
  rows 3-5 = linear   (vx, vy, vz)
"""
import numpy as np


# ── Active functions (arm-only WBC) ───────────────────────────────────────────

def damped_pinv(J: np.ndarray, damping: float = 1e-3) -> np.ndarray:
    """
    Damped pseudo-inverse: J⁺ = Jᵀ(JJᵀ + λ²I)⁻¹

    Args:
        J:        Jacobian matrix, shape (m, n) with m ≤ n
        damping:  regularization factor λ

    Returns:
        J_pinv: shape (n, m)
    """
    m = J.shape[0]
    return J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(m), np.eye(m))


def null_space_projector(J: np.ndarray, J_pinv: np.ndarray | None = None) -> np.ndarray:
    """
    Null-space projector: N = I − J⁺J

    Projects arbitrary joint velocities into the null-space of J,
    i.e. motions that do not affect the task-space velocity.

    Args:
        J:       Jacobian matrix, shape (m, n)
        J_pinv:  precomputed damped pseudo-inverse (optional)

    Returns:
        N: shape (n, n), rank n−m (or less if rank deficient)
    """
    if J_pinv is None:
        J_pinv = damped_pinv(J)
    return np.eye(J.shape[1]) - J_pinv @ J


def manipulability(J_arm: np.ndarray) -> float:
    """
    Yoshikawa manipulability: m = sqrt(det(J * J^T)).

    High value  → arm well-conditioned, far from singularity.
    Near zero   → arm near singularity.

    Args:
        J_arm: shape (6, 6)

    Returns:
        m >= 0.0
    """
    val = np.linalg.det(J_arm @ J_arm.T)
    return float(np.sqrt(max(val, 0.0)))


# ═══════════════════════════════════════════════════════════════════════════════
# Deprecated — kept for reference if Spot base re-integration is needed in future
# ═══════════════════════════════════════════════════════════════════════════════

def compute_j_base(p_ee_in_body: np.ndarray) -> np.ndarray:
    """
    Analytical Jacobian mapping Spot base velocity [vx, wz] → EE spatial velocity.

    When Spot moves at (vx, wz) in body frame, the EE velocity is:
      v_lin = [vx, 0, 0] + wz × p_ee
      v_ang = [0, 0, wz]

    Args:
        p_ee_in_body: EE position in Spot body frame, shape (3,)  [px, py, pz]

    Returns:
        J_base: shape (6, 2), Pinocchio convention [ang(3); lin(3)]
    """
    px, py, _ = p_ee_in_body
    J = np.zeros((6, 2))
    J[2, 1] = 1.0
    J[3, 0] = 1.0
    J[3, 1] = -py
    J[4, 1] = px
    return J


def compute_j_holistic(J_arm: np.ndarray, J_base: np.ndarray) -> np.ndarray:
    """Assemble holistic Jacobian J = [J_arm | J_base].  shape (6, 8)."""
    return np.hstack([J_arm, J_base])


def wbc_split(
    J_holistic, v_des, m,
    lam_arm=1.0, lam_base=1.0, damping=1e-3,
    vx_max=0.4, wz_max=0.5, q_dot_max=0.6,
) -> tuple[np.ndarray, float, float]:
    """
    Weighted damped least-squares WBC split for holistic [arm | base] Jacobian.

    Solves:  min_x  ½ xᵀWx   s.t.  J_holistic·x ≈ v_des

    Weight matrix W:  w_arm = lam_arm / (m+ε),  w_base = lam_base (constant).

    Returns (q_dot, vx, wz).
    """
    eps = 1e-4
    n_all = J_holistic.shape[1]
    n_arm = n_all - 2
    w_diag = np.array([lam_arm / (m + eps)] * n_arm + [lam_base, lam_base])
    W_inv = np.diag(1.0 / w_diag)
    n_rows = v_des.shape[0]
    A = J_holistic @ W_inv @ J_holistic.T + damping * np.eye(n_rows)
    x = W_inv @ J_holistic.T @ np.linalg.solve(A, v_des)
    q_dot = np.clip(x[:n_arm], -q_dot_max, q_dot_max)
    vx = float(np.clip(x[n_arm], -vx_max, vx_max))
    wz = float(np.clip(x[n_arm + 1], -wz_max, wz_max))
    return q_dot, vx, wz


def wbc_split_with_yaw(
    J_holistic, v_des, m, yaw_error,
    k_yaw=0.5, lam_arm=1.0, lam_base=1.0, damping=1e-3,
    vx_max=0.4, wz_max=0.5, q_dot_max=0.6,
) -> tuple[np.ndarray, float, float]:
    """
    WBC split with additional base yaw task.

    Extends J_holistic (6×8) with a 7th row [0,...,0,1] that selects wz,
    creating a 7×8 system. Desired yaw rate = k_yaw * yaw_error.

    Returns (q_dot, vx, wz).
    """
    eps = 1e-4
    n_all = J_holistic.shape[1]
    n_arm = n_all - 2
    w_diag = np.array([lam_arm / (m + eps)] * n_arm + [lam_base, lam_base])
    W_inv = np.diag(1.0 / w_diag)
    J_yaw_row = np.zeros((1, n_all))
    J_yaw_row[0, -1] = 1.0
    J_ext = np.vstack([J_holistic, J_yaw_row])
    v_des_ext = np.append(v_des, k_yaw * yaw_error)
    n_rows = v_des_ext.shape[0]
    A = J_ext @ W_inv @ J_ext.T + damping * np.eye(n_rows)
    x = W_inv @ J_ext.T @ np.linalg.solve(A, v_des_ext)
    q_dot = np.clip(x[:n_arm], -q_dot_max, q_dot_max)
    vx = float(np.clip(x[n_arm], -vx_max, vx_max))
    wz = float(np.clip(x[n_arm + 1], -wz_max, wz_max))
    return q_dot, vx, wz
