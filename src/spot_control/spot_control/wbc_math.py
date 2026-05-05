"""
wbc_math.py — Pure WBC math functions, no ROS dependencies.

Conventions:
  Spot body frame : X=forward, Y=left, Z=up
  cmd_vel         : [vx, wz]  (non-holonomic, no lateral)
  Z1 joints       : 6 DOF
  Decision var    : x = [q_dot(6), vx, wz]   shape (8,)

Pinocchio spatial velocity convention: [omega(3), v(3)]
  rows 0-2 = angular  (wx, wy, wz)
  rows 3-5 = linear   (vx, vy, vz)
"""
import numpy as np


# ── J_base ────────────────────────────────────────────────────────────────────

def compute_j_base(p_ee_in_body: np.ndarray) -> np.ndarray:
    """
    Analytical Jacobian mapping Spot base velocity [vx, wz] → EE spatial velocity.

    When Spot moves at (vx, wz) in body frame, the EE velocity is:
      v_lin = [vx, 0, 0] + wz × p_ee   (cross product in 3D)
      v_ang = [0,  0, wz]

    Expanded:
      vx_ee = vx - wz * p_ee_y
      vy_ee =      wz * p_ee_x
      vz_ee = 0
      wx_ee = 0
      wy_ee = 0
      wz_ee = wz

    Args:
        p_ee_in_body: EE position in Spot body frame, shape (3,)  [px, py, pz]

    Returns:
        J_base: shape (6, 2), Pinocchio convention [ang(3); lin(3)]
                column 0 = d/d_vx,  column 1 = d/d_wz
    """
    px, py, _ = p_ee_in_body

    J = np.zeros((6, 2))

    # Angular rows (0-2): only wz contributes to wz_ee
    J[2, 1] = 1.0       # wz_ee = wz

    # Linear rows (3-5)
    J[3, 0] = 1.0       # vx_ee = vx  ...
    J[3, 1] = -py       #         ... - wz * py
    J[4, 1] =  px       # vy_ee = wz * px
    # vz_ee = 0  (Spot stays on ground)

    return J


# ── J_holistic ────────────────────────────────────────────────────────────────

def compute_j_holistic(J_arm: np.ndarray, J_base: np.ndarray) -> np.ndarray:
    """
    Assemble holistic Jacobian J = [J_arm | J_base].

    Args:
        J_arm:  shape (6, 6)  — from Pinocchio, LOCAL_WORLD_ALIGNED
        J_base: shape (6, 2)  — from compute_j_base()

    Returns:
        J_holistic: shape (6, 8)
    """
    return np.hstack([J_arm, J_base])


# ── Manipulability ────────────────────────────────────────────────────────────

def manipulability(J_arm: np.ndarray) -> float:
    """
    Yoshikawa manipulability: m = sqrt(det(J * J^T)).

    High value  → arm well-conditioned, far from singularity.
    Near zero   → arm near singularity, prefer base motion.

    Args:
        J_arm: shape (6, 6)

    Returns:
        m >= 0.0
    """
    val = np.linalg.det(J_arm @ J_arm.T)
    return float(np.sqrt(max(val, 0.0)))


# ── WBC split ─────────────────────────────────────────────────────────────────

def wbc_split(
    J_holistic: np.ndarray,
    v_des: np.ndarray,
    m: float,
    lam_arm:    float = 1.0,
    lam_base:   float = 1.0,
    damping:    float = 1e-3,
    vx_max:     float = 0.4,
    wz_max:     float = 0.5,
    q_dot_max:  float = 0.6,
) -> tuple[np.ndarray, float, float]:
    """
    Weighted damped least-squares WBC split.

    Solves:  min_x  ½ xᵀ W x
             s.t.   J_holistic x ≈ v_des

    Weight matrix W (diagonal):
      arm DOFs  : w_arm  = lam_arm / (m + eps)
                  → high m  (well-conditioned) → low weight → arm preferred
                  → low  m  (near singular)    → high weight → base preferred
      base DOFs : w_base = lam_base  (constant)

    Solution (weighted damped pseudo-inverse):
      x = W⁻¹ Jᵀ (J W⁻¹ Jᵀ + damping·I)⁻¹ v_des

    Args:
        J_holistic : shape (6, 8)
        v_des      : desired EE spatial velocity [ang(3), lin(3)], shape (6,)
        m          : current arm manipulability scalar
        lam_arm    : base weight scale for arm DOFs
        lam_base   : weight for base DOFs
        damping    : regularisation factor
        vx_max     : Spot max forward velocity  [m/s]
        wz_max     : Spot max yaw velocity      [rad/s]
        q_dot_max  : Z1 max joint velocity      [rad/s]

    Returns:
        q_dot : arm joint velocities, shape (6,)   [rad/s]
        vx    : Spot forward velocity              [m/s]
        wz    : Spot yaw velocity                  [rad/s]
    """
    eps = 1e-4
    w_arm  = lam_arm  / (m + eps)
    w_base = lam_base

    n_all = J_holistic.shape[1]
    n_arm = n_all - 2

    w_diag = np.array([w_arm] * n_arm + [w_base, w_base])
    W_inv  = np.diag(1.0 / w_diag)

    # Weighted damped least-squares
    n_rows = v_des.shape[0]
    A = J_holistic @ W_inv @ J_holistic.T + damping * np.eye(n_rows)
    x = W_inv @ J_holistic.T @ np.linalg.solve(A, v_des)

    q_dot = np.clip(x[:n_arm],     -q_dot_max, q_dot_max)
    vx    = float(np.clip(x[n_arm],     -vx_max,    vx_max))
    wz    = float(np.clip(x[n_arm + 1], -wz_max,    wz_max))

    return q_dot, vx, wz


# ── WBC split with yaw task ───────────────────────────────────────────────────

def wbc_split_with_yaw(
    J_holistic: np.ndarray,
    v_des: np.ndarray,
    m: float,
    yaw_error: float,
    k_yaw:      float = 0.5,
    lam_arm:    float = 1.0,
    lam_base:   float = 1.0,
    damping:    float = 1e-3,
    vx_max:     float = 0.4,
    wz_max:     float = 0.5,
    q_dot_max:  float = 0.6,
) -> tuple[np.ndarray, float, float]:
    """
    WBC split with an additional base yaw task.

    Extends J_holistic (6×8) with a 7th row that selects wz directly,
    creating a 7×8 system. The QP balances EE position tracking (rows 1-6)
    against base yaw tracking (row 7).

    The yaw task row [0,0,0,0,0,0,0,1] maps u[7]=wz → desired yaw rate.
    The desired yaw rate is k_yaw * yaw_error [rad/s].

    Args:
        J_holistic : shape (6, 8)
        v_des      : desired EE spatial velocity [ang(3), lin(3)], shape (6,)
        m          : current arm manipulability scalar
        yaw_error  : θ_des - θ_current, wrapped to (-π, π) [rad]
        k_yaw      : proportional gain for yaw task [rad/s per rad]
        (other args same as wbc_split)

    Returns:
        q_dot, vx, wz  (same as wbc_split)
    """
    eps = 1e-4
    w_arm  = lam_arm  / (m + eps)
    w_base = lam_base

    n_all = J_holistic.shape[1]
    n_arm = n_all - 2

    w_diag = np.array([w_arm] * n_arm + [w_base, w_base])
    W_inv  = np.diag(1.0 / w_diag)

    # Extend Jacobian: add yaw selection row — wz is always the last column
    J_yaw_row = np.zeros((1, n_all))
    J_yaw_row[0, -1] = 1.0
    J_ext = np.vstack([J_holistic, J_yaw_row])

    v_des_ext = np.append(v_des, k_yaw * yaw_error)

    # Weighted damped least-squares
    n_rows = v_des_ext.shape[0]
    A = J_ext @ W_inv @ J_ext.T + damping * np.eye(n_rows)
    x = W_inv @ J_ext.T @ np.linalg.solve(A, v_des_ext)

    q_dot = np.clip(x[:n_arm],     -q_dot_max, q_dot_max)
    vx    = float(np.clip(x[n_arm],     -vx_max,    vx_max))
    wz    = float(np.clip(x[n_arm + 1], -wz_max,    wz_max))

    return q_dot, vx, wz
