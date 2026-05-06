"""
Shared orientation & transform utilities for TERESA.

Frame conventions:
  - world / link00: X=toward patient, Y=head→feet, Z=right→left
  - EE:       link06 end-effector
  - Spot body: X=forward, Y=left, Z=up
  - Optical:   X=right, Y=down, Z=forward (ROS REP-104)

All functions are pure math — no ROS node dependencies.
"""

import math

import numpy as np
from tf_transformations import quaternion_from_matrix, quaternion_matrix


def compute_ee_orientation(
    x_ee: np.ndarray,
    home_orientation,
):
    """
    Compute EE orientation with X_ee = x_ee, Y_ee near home configuration.

    Uses Gram-Schmidt to project home Y axis orthogonal to X_ee,
    with fallback to home Z if home Y is nearly parallel to X_ee.
    This ensures a clean approach orientation with zero twist.

    Args:
        x_ee: (3,) array — desired EE X direction (normalised internally).
        home_orientation: [qx, qy, qz, qw] quaternion of the home pose.

    Returns:
        [x, y, z, w] quaternion as numpy array.
    """
    x_ee = x_ee / np.linalg.norm(x_ee)
    R_home = quaternion_matrix(home_orientation)[:3, :3]

    y_ref = R_home[:, 1]
    y_ee = y_ref - np.dot(y_ref, x_ee) * x_ee
    y_norm = float(np.linalg.norm(y_ee))
    if y_norm < 1e-3:
        y_ref = R_home[:, 2]
        y_ee = y_ref - np.dot(y_ref, x_ee) * x_ee
        y_norm = float(np.linalg.norm(y_ee))
    y_ee /= y_norm

    z_ee = np.cross(x_ee, y_ee)

    T = np.eye(4)
    T[:3, 0] = x_ee
    T[:3, 1] = y_ee
    T[:3, 2] = z_ee
    return quaternion_from_matrix(T)


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product: q1 * q2 (q = [x, y, z, w])."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def compute_ee_orientation_minrot(
    x_ee: np.ndarray,
    home_orientation,
):
    """
    Compute EE orientation by minimum rotation from home X to x_ee.

    Unlike Gram-Schmidt (which constrains Y_ee near home Y and can force
    wrist twist when x_ee points downward), this rotates the entire home
    orientation along the shortest arc to align home X with x_ee.
    The wrist configuration (joints 4-5-6) stays near the home pose.

    Args:
        x_ee: (3,) array — desired EE X direction (normalised internally).
        home_orientation: [qx, qy, qz, qw] quaternion of the home pose.

    Returns:
        [x, y, z, w] quaternion as numpy array.
    """
    x_ee = x_ee / np.linalg.norm(x_ee)
    R_home = quaternion_matrix(home_orientation)[:3, :3]
    x_home = R_home[:, 0]

    axis = np.cross(x_home, x_ee)
    sin_a = float(np.linalg.norm(axis))
    cos_a = float(np.dot(x_home, x_ee))

    if sin_a < 1e-6:
        return np.array(home_orientation, dtype=float)

    axis /= sin_a
    angle = math.atan2(sin_a, cos_a)

    half = angle * 0.5
    s = math.sin(half)
    q_rot = np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half)])

    q_home = np.array(home_orientation, dtype=float)
    return _quat_multiply(q_rot, q_home)


def quat_to_rot(q) -> np.ndarray:
    """
    geometry_msgs Quaternion → (3, 3) rotation matrix.

    Args:
        q: object with .x, .y, .z, .w float attributes.

    Returns:
        (3, 3) np.ndarray.
    """
    R = quaternion_matrix([q.x, q.y, q.z, q.w])
    return R[:3, :3]


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """
    (3, 3) rotation matrix → [x, y, z, w] quaternion.

    Args:
        R: (3, 3) np.ndarray.

    Returns:
        [x, y, z, w] numpy array.
    """
    M = np.eye(4)
    M[:3, :3] = R
    return quaternion_from_matrix(M)


def normalize_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return float((a + math.pi) % (2 * math.pi) - math.pi)
