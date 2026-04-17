import math
import pytest
from spot_control.spot_goal_navigator import NavState, compute_cmd_vel


# ── Fixtures ─────────────────────────────────────────────────────────────────

class Params:
    angular_speed_max = 0.5
    linear_speed_max  = 0.4
    angle_threshold   = 0.15
    goal_tolerance    = 0.3
    kp_ang            = 1.0
    kp_lin            = 0.5


P = Params()


# ── ROTATING state ────────────────────────────────────────────────────────────

def test_rotating_turns_toward_goal_on_left():
    """Goal to the left (positive angle) → positive angular.z."""
    twist = compute_cmd_vel(dx=1.0, dy=0.5, state=NavState.ROTATING, params=P)
    assert twist.linear.x == 0.0
    assert twist.angular.z > 0.0


def test_rotating_turns_toward_goal_on_right():
    """Goal to the right (negative angle) → negative angular.z."""
    twist = compute_cmd_vel(dx=1.0, dy=-0.5, state=NavState.ROTATING, params=P)
    assert twist.linear.x == 0.0
    assert twist.angular.z < 0.0


def test_rotating_clamps_to_max_speed():
    """Large angle error → clamped to angular_speed_max."""
    twist = compute_cmd_vel(dx=0.0, dy=5.0, state=NavState.ROTATING, params=P)
    assert abs(twist.angular.z) <= P.angular_speed_max


def test_rotating_zero_when_aligned():
    """Goal directly ahead → zero angular.z."""
    twist = compute_cmd_vel(dx=1.0, dy=0.0, state=NavState.ROTATING, params=P)
    assert twist.angular.z == 0.0
    assert twist.linear.x == 0.0


# ── DRIVING state ─────────────────────────────────────────────────────────────

def test_driving_moves_forward():
    """Goal ahead → positive linear.x."""
    twist = compute_cmd_vel(dx=1.0, dy=0.0, state=NavState.DRIVING, params=P)
    assert twist.linear.x > 0.0


def test_driving_clamps_linear_to_max():
    """Far goal → clamped to linear_speed_max."""
    twist = compute_cmd_vel(dx=20.0, dy=0.0, state=NavState.DRIVING, params=P)
    assert twist.linear.x <= P.linear_speed_max


def test_driving_small_angular_correction():
    """Slight drift → angular correction ≤ half angular_speed_max."""
    twist = compute_cmd_vel(dx=1.0, dy=0.2, state=NavState.DRIVING, params=P)
    assert abs(twist.angular.z) <= P.angular_speed_max / 2.0


def test_driving_no_negative_linear():
    """linear.x never negative (no reversing)."""
    twist = compute_cmd_vel(dx=-1.0, dy=0.0, state=NavState.DRIVING, params=P)
    assert twist.linear.x >= 0.0


# ── IDLE / STOPPED states ─────────────────────────────────────────────────────

def test_idle_returns_zero_twist():
    twist = compute_cmd_vel(dx=1.0, dy=0.5, state=NavState.IDLE, params=P)
    assert twist.linear.x == 0.0
    assert twist.angular.z == 0.0


def test_stopped_returns_zero_twist():
    twist = compute_cmd_vel(dx=1.0, dy=0.5, state=NavState.STOPPED, params=P)
    assert twist.linear.x == 0.0
    assert twist.angular.z == 0.0
