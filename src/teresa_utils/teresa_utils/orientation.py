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
