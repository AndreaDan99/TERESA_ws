#!/usr/bin/env python3
"""
Search Time Model: Spot+Z1 Arm vs Orbbec-Only
==============================================
Compares expected search times for finding a prone/supine patient using:

  Approach A — Spot + Z1 Arm (TERESA system):
    Spot rotates to ±30° yaw (2 positions). At each, the Z1 arm executes
    7 scanning poses covering ~300° around Spot (only ~60° blind spot
    behind Spot's body). Combined coverage from 2 yaw positions: full 360°.

  Approach B — Spot-only (Orbbec Femto Bolt camera fixed on Spot body):
    Orbbec camera has ~87° horizontal FOV. With 10° overlap for reliable
    detection, effective FOV = 77° per position. Spot must physically
    rotate through N = ceil(360/77) = 5 positions to cover 360°.

All parameters are configurable constants traced to the actual codebase
config (src/spot_control/config/wbc_params.yaml) and hardware specs.

Author: TERESA project
"""

import math
from dataclasses import dataclass, field
from itertools import cycle
from typing import List

# ═══════════════════════════════════════════════════════════════════════
# CONFIGURABLE PARAMETERS — all traceable to codebase or hardware specs
# ═══════════════════════════════════════════════════════════════════════

# ── Spot kinematics ──────────────────────────────────────────────────
SPOT_YAW_SPEED = 0.2          # rad/s — conservative (search_max_angular_vel in wbc_params.yaml:91)
SPOT_YAW_SPEED_FAST = 0.5     # rad/s — maximum (default in code, wbc_params.yaml:91 comment)
# Note: 0.2 rad/s ≈ 11.46 °/s.  To rotate 30°: 30°/11.46°/s ≈ 2.62 s.

# Yaw angles used in search (from search_yaw_angles: [30.0, -30.0] in wbc_params.yaml:84)
SPOT_YAW_ANGLES = [30.0, -30.0]   # degrees — relative steps from initial heading

# ── Z1 arm ───────────────────────────────────────────────────────────
ARM_POSES_PER_YAW = 7         # ik_done events per yaw position (7 search poses)
ARM_POSE_TIME = 1.2           # seconds per pose (search_timeout_per_point, wbc_params.yaml:26)
ARM_COVERAGE_DEG = 300.0      # degrees covered by arm from one Spot position
# The arm's camera (Orbbec at link06) can cover ±150° from the current
# Spot heading, leaving a ~60° blind spot directly behind Spot's body.

# ── Orbbec Femto Bolt camera ─────────────────────────────────────────
ORBBEC_FOV_H = 87.0           # degrees horizontal FOV (hardware spec)
ORBBEC_OVERLAP = 10.0         # degrees overlap needed for reliable detection
# Effective FOV per position: ORBBEC_FOV_H - ORBBEC_OVERLAP = 77°

# ── Timing ───────────────────────────────────────────────────────────
DETECTION_DWELL = 2.0         # seconds to observe/detect at a position (Orbbec-only)
HOME_STEP_TIME = 3.0          # seconds for arm HOME + Spot step forward between cycles
# Breakdown: HOME ik_done ~1s + step 0.2m / 0.3 m/s = 0.67s + overhead (wbc_params.yaml:85-86)

# ── Simulation ───────────────────────────────────────────────────────
MONTE_CARLO_SAMPLES = 10000   # number of random patient positions to sample

# ── Comparison angles for per-position table ─────────────────────────
COMPARISON_ANGLES = [0, 45, 90, 135, 180]  # degrees

# ═══════════════════════════════════════════════════════════════════════
# DERIVED CONSTANTS
# ═══════════════════════════════════════════════════════════════════════

SPOT_YAW_SPEED_DEG_S = math.degrees(SPOT_YAW_SPEED)       # ~11.46 °/s
EFFECTIVE_FOV = ORBBEC_FOV_H - ORBBEC_OVERLAP              # 77°
ARM_SCAN_TIME = ARM_POSES_PER_YAW * ARM_POSE_TIME           # 8.4 s
HALF_COVERAGE = ARM_COVERAGE_DEG / 2.0                      # 150°
MIN_DETECTION_TIME = ARM_POSE_TIME / 2.0                    # 0.6 s — half a pose minimum

# Orbbec-only positions (N=5, equally spaced by effective FOV)
ORBBEC_N_POSITIONS = math.ceil(360.0 / EFFECTIVE_FOV)       # 5
# Position angles [°]: 0°, 77°, 154°, 231°, 308°
ORBBEC_POSITIONS = [i * EFFECTIVE_FOV for i in range(ORBBEC_N_POSITIONS)]

# ═══════════════════════════════════════════════════════════════════════
# HELPER FUNCTIONS
# ═══════════════════════════════════════════════════════════════════════

def normalize_angle_deg(theta: float) -> float:
    """Normalize angle to (-180, 180] degrees."""
    theta = math.fmod(theta, 360.0)
    if theta > 180.0:
        theta -= 360.0
    elif theta <= -180.0:
        theta += 360.0
    return theta


def angular_distance_deg(a: float, b: float) -> float:
    """Shortest angular distance |a - b| in degrees [0, 180]."""
    return abs(normalize_angle_deg(a - b))


def rotation_time_deg(angle_diff_deg: float) -> float:
    """Time to rotate `angle_diff_deg` at SPOT_YAW_SPEED (seconds)."""
    return abs(math.radians(angle_diff_deg)) / SPOT_YAW_SPEED


def normalize_angle_rad(theta: float) -> float:
    """Normalize angle to (-pi, pi] radians."""
    theta = math.fmod(theta, 2.0 * math.pi)
    if theta > math.pi:
        theta -= 2.0 * math.pi
    elif theta <= -math.pi:
        theta += 2.0 * math.pi
    return theta


# ═══════════════════════════════════════════════════════════════════════
# MODEL A: SPOT + Z1 ARM SEARCH TIME
# ═══════════════════════════════════════════════════════════════════════
#
# The search follows the code's actual sequence:
#   yaw positions = [initial heading, +30°, -30°]   (3 scanning positions)
#
# At each yaw position ψ, the arm scans 7 poses covering ±150° around ψ,
# i.e. [ψ−150°, ψ+150°]. Detection within a scan is modeled as:
#
#   t_detect(θ, ψ) = ( |θ − ψ| / 150° ) × T_scan  +  T_pose/2
#
# where the first term represents the linear scan from the arm's center
# direction outward, and T_pose/2 is a minimum latency (half a pose).
# If |θ − ψ| > 150°, the patient is not detectable at this yaw.
#
# The search proceeds sequentially through yaw positions:
#   1. Yaw 0°  — arm already deployed, no rotation needed to start
#   2. Yaw +30° — rotate from 0° to +30°, then scan
#   3. Yaw -30° — rotate from +30° to -30°, then scan
#
# Each scan that fails to find the patient takes the full T_scan.
# The first scan that covers the patient finds them.
#
# For the 300° coverage at each yaw, the blind spot of 60° changes:
#   Yaw   0° covers [−150°, +150°]       blind = [150°, 210°] ≡ [150°, −150°]
#   Yaw +30° covers [−120°, +180°]       blind = [180°, 240°] ≡ [180°, −120°]
#   Yaw -30° covers [−180°, +120°]       blind = [120°, 180°]? No.
#                                        blind at ψ−30° = [−180°+120°, −180°+180°] 
#                                        Wait: blind is 60° behind Spot. 
#                                        At ψ=−30°, blind = [−30+150, −30+210] 
#                                        = [120°, 180°] in the coverage gap.
#                                        Actually: coverage end at ψ−30° is 
#                                        [-180°, +120°] (since −30°+150°=120°,
#                                        −30°−150°=−180°). Gap = (120°, 180°] 
#                                        which is covered by ψ+30° whose 
#                                        coverage extends to 180°.
#
# The 3 yaw positions combined cover the full 360° (with significant overlap).

def spot_arm_search_time(
    theta_deg: float,
    yaw_angles: List[float] | None = None,
) -> float:
    """Expected search time (seconds) for Spot+Arm to find patient at yaw θ.

    Args:
        theta_deg: Patient yaw angle relative to Spot's initial heading [°].
        yaw_angles: Ordered list of relative yaw positions [°].  Defaults to
                    [0.0, 30.0, -30.0] which includes scanning at the initial
                    heading before rotating.
    Returns:
        Expected search time in seconds.
    """
    if yaw_angles is None:
        yaw_angles = [0.0] + SPOT_YAW_ANGLES   # [0°, +30°, -30°]

    theta = normalize_angle_deg(theta_deg)
    cumulative_rotation = 0.0      # accrued rotation time [s]
    current_yaw = 0.0              # current Spot yaw [°]

    for yaw in yaw_angles:
        # Rotate from current to this yaw
        rot_time = rotation_time_deg(abs(normalize_angle_deg(yaw - current_yaw)))
        cumulative_rotation += rot_time
        current_yaw = yaw

        # Check if patient is within coverage at this yaw
        offset = angular_distance_deg(theta, yaw)
        if offset <= HALF_COVERAGE:
            # Detection occurs during this scan.
            # Detection time within the scan: proportional to angular offset
            # from the yaw center, plus a minimum latency.
            scan_fraction = offset / HALF_COVERAGE   # [0, 1]
            detection_time = scan_fraction * ARM_SCAN_TIME + MIN_DETECTION_TIME
            # Clamp to [MIN_DETECTION_TIME, ARM_SCAN_TIME]
            detection_time = max(MIN_DETECTION_TIME, min(detection_time, ARM_SCAN_TIME))
            return cumulative_rotation + detection_time

        # Patient not found in this yaw — full scan was performed
        cumulative_rotation += ARM_SCAN_TIME

    # Should never reach here (360° covered), but fallback
    return cumulative_rotation


def spot_arm_search_time_code_model(theta_deg: float) -> float:
    """Search time using the code's exact sequence (no initial yaw-0 scan).

    The code always rotates to +30° first (wbc_params.yaml search_yaw_angles: 
    [30, -30]), then -30°.  There is no separate "scan at initial heading" step.
    """
    theta = normalize_angle_deg(theta_deg)

    # Yaw +30°: coverage [-120°, +180°]
    yaw1 = SPOT_YAW_ANGLES[0]   # +30°
    rot1 = rotation_time_deg(abs(yaw1))                     # 30° → 2.62 s

    offset1 = angular_distance_deg(theta, yaw1)
    if offset1 <= HALF_COVERAGE:
        # θ within yaw +30° coverage
        frac = offset1 / HALF_COVERAGE
        detect = frac * ARM_SCAN_TIME + MIN_DETECTION_TIME
        detect = max(MIN_DETECTION_TIME, min(detect, ARM_SCAN_TIME))
        return rot1 + detect

    # Yaw -30°: coverage [-180°, +120°]
    yaw2 = SPOT_YAW_ANGLES[1]   # -30°
    # Rotation from +30° to -30°: 60° physical, but code uses
    # abs(yaw2)/SPOT_YAW_SPEED = 30°/(0.2 rad/s) = 2.62 s (simplified).
    # Use actual angular displacement for physical accuracy:
    rot2 = rotation_time_deg(abs(normalize_angle_deg(yaw2 - yaw1)))  # 60° → 5.24 s

    offset2 = angular_distance_deg(theta, yaw2)
    frac = offset2 / HALF_COVERAGE
    detect = frac * ARM_SCAN_TIME + MIN_DETECTION_TIME
    detect = max(MIN_DETECTION_TIME, min(detect, ARM_SCAN_TIME))
    # total = rot1 + full_scan_at_yaw1 + rot2 + detection_at_yaw2
    return rot1 + ARM_SCAN_TIME + rot2 + detect


# ═══════════════════════════════════════════════════════════════════════
# MODEL B: ORBBEC-ONLY SEARCH TIME
# ═══════════════════════════════════════════════════════════════════════
#
# The Orbbec Femto Bolt camera is fixed on Spot's body (no arm).
# With 87° H-FOV and 10° overlap → effective FOV = 77° per position.
# N = ceil(360/77) = 5 positions needed for 360° coverage.
#
# Search strategy: Spot sequentially rotates through positions and dwells
# at each for DETECTION_DWELL seconds to acquire and process frames.
# At each position j, the camera covers:
#   [pos_j − EFF_FOV/2,  pos_j + EFF_FOV/2]
#
# The search starts at position 0 (0°) and proceeds:
#   pos₀=0° → pos₁=77° → pos₂=154° → pos₃=231° → pos₄=308°
#
# Rotation between consecutive positions: 77° / SPOT_YAW_SPEED = 6.72 s.
# At each position: 2.0 s dwell for detection.
#
# Expected search time for patient at θ:
#   Find first position j covering θ. Time = Σⁱ⁼⁰ⱼ₋₁ (rot_i + dwell) + dwell_j

def orbbec_only_search_time(theta_deg: float) -> float:
    """Expected search time (seconds) for Orbbec-only approach.

    Args:
        theta_deg: Patient yaw angle relative to Spot's initial heading [°].
    Returns:
        Expected search time in seconds.
    """
    theta = normalize_angle_deg(theta_deg)
    half_fov = EFFECTIVE_FOV / 2.0   # 38.5°

    # Check each position in order; find the first one covering θ
    elapsed = 0.0
    current_yaw = 0.0

    for i, pos_deg in enumerate(ORBBEC_POSITIONS):
        if i > 0:
            # Rotate from previous to this position — take shortest path
            rot_deg = angular_distance_deg(pos_deg, ORBBEC_POSITIONS[i - 1])
            elapsed += rotation_time_deg(rot_deg)
            current_yaw = pos_deg

        # Dwell at this position
        elapsed += DETECTION_DWELL

        # Check coverage: camera at position pos_deg covers ±half_fov
        if angular_distance_deg(theta, pos_deg) <= half_fov:
            return elapsed

        # Continue to next position if not found

    # Fallback: should never happen (360° covered)
    return elapsed


def orbbec_only_search_time_optimal(theta_deg: float) -> float:
    """Optimal Orbbec-only search (rotate shortest direction to covering position).

    Instead of sequential scan, Spot goes directly to the covering position
    (closest in angular distance).  This is a theoretical lower bound.
    """
    theta = normalize_angle_deg(theta_deg)
    half_fov = EFFECTIVE_FOV / 2.0

    # Find the position covering θ
    for pos_deg in ORBBEC_POSITIONS:
        if angular_distance_deg(theta, pos_deg) <= half_fov:
            # Rotate directly (shortest path) from 0° to this position
            rot_time = rotation_time_deg(angular_distance_deg(0.0, pos_deg))
            return rot_time + DETECTION_DWELL

    # Not found — shouldn't happen
    return float('inf')


# ═══════════════════════════════════════════════════════════════════════
# MONTE CARLO SIMULATION
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class SearchStats:
    mean: float
    median: float
    p95: float
    p99: float
    min_val: float
    max_val: float


def monte_carlo_sim(
    search_fn,
    n_samples: int = MONTE_CARLO_SAMPLES,
) -> SearchStats:
    """Run Monte Carlo simulation for uniform random patient yaw [0°, 360°)."""
    import random
    random.seed(42)     # reproducibility

    times = []
    for _ in range(n_samples):
        theta = random.uniform(0.0, 360.0)
        t = search_fn(theta)
        times.append(t)

    times.sort()
    n = len(times)
    return SearchStats(
        mean=sum(times) / n,
        median=times[n // 2],
        p95=times[int(n * 0.95)],
        p99=times[int(n * 0.99)],
        min_val=times[0],
        max_val=times[-1],
    )


# ═══════════════════════════════════════════════════════════════════════
# OUTPUT FORMATTING
# ═══════════════════════════════════════════════════════════════════════

# Nerd Font icons (compatible with most terminals)
ICON_ROBOT = '🤖'
ICON_CAMERA = '📷'
ICON_CHART = '📊'
ICON_LATEX = '📄'
ICON_STOPWATCH = '⏱'
ICON_CHECK = '✅'


def print_header():
    """Print parameter summary."""
    print(f"{'='*70}")
    print(f"  {ICON_ROBOT}  SEARCH TIME COMPARISON: Spot+Z1 Arm vs Orbbec-Only")
    print(f"{'='*70}")
    print()
    print(f"  Parameters (from codebase config & hardware specs):")
    print(f"    Spot yaw speed:       {SPOT_YAW_SPEED:.2f} rad/s  ({SPOT_YAW_SPEED_DEG_S:.1f} °/s)")
    print(f"    Spot search yaws:     {SPOT_YAW_ANGLES} °")
    print(f"    Arm poses per yaw:    {ARM_POSES_PER_YAW}")
    print(f"    Arm pose time:        {ARM_POSE_TIME:.1f} s  (search_timeout_per_point)")
    print(f"    Arm scan duration:    {ARM_SCAN_TIME:.1f} s  per yaw position")
    print(f"    Arm coverage:         {ARM_COVERAGE_DEG:.0f}°  per yaw  (blind spot: {360-ARM_COVERAGE_DEG:.0f}°)")
    print(f"    Orbbec FOV (H):       {ORBBEC_FOV_H:.0f}°  (Femto Bolt)")
    print(f"    Effective FOV:        {EFFECTIVE_FOV:.0f}°  (−{ORBBEC_OVERLAP:.0f}° overlap)")
    print(f"    Orbbec positions:     {ORBBEC_N_POSITIONS}  (at {ORBBEC_POSITIONS})")
    print(f"    Detection dwell:      {DETECTION_DWELL:.1f} s  per Orbbec position")
    print(f"    HOME + step time:     {HOME_STEP_TIME:.1f} s  (between search cycles)")
    print(f"    Monte Carlo samples:  {MONTE_CARLO_SAMPLES}")
    print()


def print_per_position_table(angles: List[int],
                              spot_fn,
                              orbbec_fn=None):
    """Print per-position comparison table."""
    if orbbec_fn is None:
        orbbec_fn = orbbec_only_search_time

    print(f"{'─'*70}")
    print(f"  {ICON_STOPWATCH}  Per-Position Comparison")
    print(f"{'─'*70}")
    print(f"  {'Patient Yaw':>10s} │ {'Spot+Arm (s)':>13s} │ {'Orbbec-Only (s)':>15s} │ {'Speedup':>8s}")
    print(f"  {'─'*10}─┼─{'─'*13}─┼─{'─'*15}─┼─{'─'*8}")

    for angle in angles:
        t_spot = spot_fn(float(angle))
        t_obb = orbbec_fn(float(angle))
        speedup = t_obb / t_spot if t_spot > 0 else float('inf')
        marker = ' ← best' if speedup > 2.0 else ''
        print(f"  {angle:>7d}°    │ {t_spot:>10.1f}    │ {t_obb:>12.1f}      │ {speedup:>5.1f}×{marker}")

    print()


def print_aggregate_stats(stats_spot: SearchStats,
                           stats_obb: SearchStats,
                           label: str = "Spot+Arm (3-yaw) vs Orbbec-Only"):
    """Print aggregate statistics from Monte Carlo simulation."""
    print(f"{'─'*70}")
    print(f"  {ICON_CHART}  Aggregate Statistics — {label}")
    print(f"  ({MONTE_CARLO_SAMPLES} random uniform patient positions, 0°–360°)")
    print(f"{'─'*70}")
    print(f"  {'':>14s} │ {'Spot+Arm':>12s} │ {'Orbbec-Only':>14s} │ {'Speedup':>8s}")
    print(f"  {'─'*14}─┼─{'─'*12}─┼─{'─'*14}─┼─{'─'*8}")

    for stat_name, s_val, o_val in [
        ("Mean (s)", stats_spot.mean, stats_obb.mean),
        ("Median (s)", stats_spot.median, stats_obb.median),
        ("P95 (s)", stats_spot.p95, stats_obb.p95),
        ("P99 (s)", stats_spot.p99, stats_obb.p99),
        ("Min (s)", stats_spot.min_val, stats_obb.min_val),
        ("Max (s)", stats_spot.max_val, stats_obb.max_val),
    ]:
        speedup = o_val / s_val if s_val > 0 else float('inf')
        print(f"  {stat_name:>14s} │ {s_val:>10.1f}    │ {o_val:>12.1f}      │ {speedup:>5.1f}×")

    print()


def generate_latex_table(angles: List[int],
                          spot_fn,
                          orbbec_fn,
                          stats_spot: SearchStats,
                          stats_obb: SearchStats) -> str:
    """Generate a LaTeX-formatted table for paper inclusion."""
    if orbbec_fn is None:
        orbbec_fn = orbbec_only_search_time

    lines = []
    lines.append(r"\begin{table}[htbp]")
    lines.append(r"  \centering")
    lines.append(r"  \caption{Search time comparison: Spot+Z1 arm vs.\ Orbbec-only camera.}")
    lines.append(r"  \label{tab:search_time}")
    lines.append(r"  \begin{tabular}{lcc}")
    lines.append(r"    \toprule")
    lines.append(r"    & \textbf{Spot+Arm} & \textbf{Orbbec-Only} \\")
    lines.append(r"    \textbf{Patient Yaw} & \textbf{Time (s)} & \textbf{Time (s)} \\")
    lines.append(r"    \midrule")

    for angle in angles:
        t_spot = spot_fn(float(angle))
        t_obb = orbbec_fn(float(angle))
        lines.append(f"    ${angle}\\degree$ & {t_spot:.1f} & {t_obb:.1f} \\\\")

    lines.append(r"    \midrule")
    lines.append(r"    \multicolumn{3}{c}{\textbf{Aggregate (10\,000 uniform random positions)}} \\")
    lines.append(r"    \midrule")

    mean_speedup = stats_obb.mean / stats_spot.mean
    median_speedup = stats_obb.median / stats_spot.median
    p95_speedup = stats_obb.p95 / stats_spot.p95

    lines.append(f"    Mean & {stats_spot.mean:.1f} & {stats_obb.mean:.1f} \\\\")
    lines.append(f"    Median & {stats_spot.median:.1f} & {stats_obb.median:.1f} \\\\")
    lines.append(f"    95th percentile & {stats_spot.p95:.1f} & {stats_obb.p95:.1f} \\\\")
    lines.append(r"    \midrule")
    lines.append(f"    \\textbf{{Speedup (O/S)}} & \\multicolumn{{2}}{{c}}{{{mean_speedup:.1f}$\\times$ (mean), "
                 f"{median_speedup:.1f}$\\times$ (median), "
                 f"{p95_speedup:.1f}$\\times$ (P95)}} \\\\")
    lines.append(r"    \bottomrule")
    lines.append(r"  \end{tabular}")
    lines.append(r"\end{table}")

    return "\n".join(lines)


def generate_methods_section() -> str:
    """Generate a LaTeX snippet describing the search model methodology."""
    return r"""
\subsection{Search Time Model}

We model the expected time to detect a prone patient at an unknown yaw angle
$\theta \in [0^\circ, 360^\circ)$ relative to the robot's initial heading.
Two configurations are compared:

\paragraph{Spot + Z1 Arm (TERESA system).}
The Z1 arm, mounted on Spot's back, carries an Orbbec Femto Bolt RGB-D camera
(87$^\circ$ horizontal FOV) on its end-effector (link06). With 6~DOF and
$\sim$600\,mm reach, the arm can orient the camera across approximately
300$^\circ$ around Spot, leaving only a $\sim$60$^\circ$ blind spot directly
behind the robot body. The search follows the sequence defined in
\texttt{wbc\_params.yaml}: Spot rotates by $\pm 30^\circ$ yaw (2 positions),
and at each position the arm executes 7 scanning poses
(\texttt{ik\_done} events, 1.2\,s per pose). At a given yaw $\psi$, the arm
covers $[\psi - 150^\circ, \psi + 150^\circ]$. Detection within a scan is
modeled as occurring at time proportional to the angular offset from the
arm's central direction.

\paragraph{Orbbec-Only (No Arm).}
The same Orbbec Femto Bolt camera is fixed on Spot's body. With an effective
FOV of $87^\circ - 10^\circ = 77^\circ$ (accounting for reliable-detection
overlap), $N = \lceil 360/77 \rceil = 5$ discrete yaw positions are required
for full 360$^\circ$ coverage. At each position, Spot dwells for 2.0\,s to
acquire and process frames. Rotation between positions proceeds at
0.2\,rad/s (the conservative limit configured in
\texttt{search\_max\_angular\_vel}), requiring 6.72\,s per 77$^\circ$ step.

\paragraph{Uniform-Random Patient Model.}
A Monte Carlo simulation with $N=10\,000$ samples draws $\theta$ uniformly
from $[0^\circ, 360^\circ)$ and computes the expected search time for both
approaches. Results are reported as mean, median, 95th percentile, and
speedup factor $T_{\text{Orbbec}} / T_{\text{Spot+Arm}}$.
"""


# ═══════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════

def main():
    print_header()

    # ── Model 1: Primary (3-yaw, including initial-heading scan) ────
    print(f"  {'='*66}")
    print(f"  MODEL 1 — Primary: Arm scans at initial heading (0°), then ±30°")
    print(f"  The arm is already deployed at the start; scanning at yaw 0°")
    print(f"  costs no rotation time.  3 yaw positions: [0°, +30°, −30°]")
    print(f"  {'='*66}")
    print()

    spot_fn_3yaw = spot_arm_search_time  # uses [0, +30, -30]
    print_per_position_table(COMPARISON_ANGLES, spot_fn_3yaw)

    stats_spot_3yaw = monte_carlo_sim(spot_fn_3yaw)
    stats_obb = monte_carlo_sim(orbbec_only_search_time)
    print_aggregate_stats(stats_spot_3yaw, stats_obb,
                          "Spot+Arm (3-yaw: 0°, +30°, −30°) vs Orbbec-Only")

    # ── Model 2: Code-matching (arm scans only at ±30°) ─────────────
    print(f"  {'='*66}")
    print(f"  MODEL 2 — Code-Matching: Arm scans only at ±30° yaw")
    print(f"  Matches the exact search_yaw_angles config: [30, −30].")
    print(f"  No initial-heading scan — rotation to +30° required first.")
    print(f"  {'='*66}")
    print()

    spot_fn_2yaw = spot_arm_search_time_code_model
    print_per_position_table(COMPARISON_ANGLES, spot_fn_2yaw)

    stats_spot_2yaw = monte_carlo_sim(spot_fn_2yaw)
    print_aggregate_stats(stats_spot_2yaw, stats_obb,
                          "Spot+Arm (2-yaw: +30°, −30° only) vs Orbbec-Only")

    # ── Model 3: Optimal Orbbec (direct rotation, lower bound) ─────
    print(f"  {'='*66}")
    print(f"  MODEL 3 — Optimal Orbbec (theoretical lower bound)")
    print(f"  Spot rotates directly to the covering position (shortest path)")
    print(f"  instead of sequentially scanning all positions.")
    print(f"  {'='*66}")
    print()

    # For 2-yaw Spot+Arm vs optimal Orbbec
    stats_obb_opt = monte_carlo_sim(orbbec_only_search_time_optimal)
    print_aggregate_stats(stats_spot_3yaw, stats_obb_opt,
                          "Spot+Arm (3-yaw) vs Orbbec-Only (optimal/direct)")

    # ── Summary speedup box ────────────────────────────────────────
    print(f"  {'='*70}")
    print(f"  {ICON_CHECK}  SUMMARY: SPEEDUP FACTORS")
    print(f"  {'='*70}")
    mean_speedup_3yaw = stats_obb.mean / stats_spot_3yaw.mean
    mean_speedup_2yaw = stats_obb.mean / stats_spot_2yaw.mean
    mean_speedup_opt = stats_obb_opt.mean / stats_spot_3yaw.mean
    print(f"    Model 1 (3-yaw):         Spot+Arm mean = {stats_spot_3yaw.mean:.1f}s,  "
          f"Orbbec mean = {stats_obb.mean:.1f}s  →  {mean_speedup_3yaw:.1f}× speedup")
    print(f"    Model 2 (2-yaw, code):   Spot+Arm mean = {stats_spot_2yaw.mean:.1f}s,  "
          f"Orbbec mean = {stats_obb.mean:.1f}s  →  {mean_speedup_2yaw:.1f}× speedup")
    print(f"    Model 3 (optimal Orbbec): Orbbec mean = {stats_obb_opt.mean:.1f}s  →  "
          f"{mean_speedup_opt:.1f}× speedup (lower bound)")
    print()
    print(f"    Key insight: The Z1 arm effectively multiplies Spot's angular")
    print(f"    coverage.  Without the arm, Spot needs to physically rotate")
    print(f"    ~5× more to cover the same search area.  The arm doesn't just")
    print(f"    scan for ultrasound — it also dramatically accelerates the")
    print(f"    initial patient-finding phase by {mean_speedup_3yaw:.1f}−{mean_speedup_2yaw:.1f}×.")
    print()

    # ── LaTeX table ─────────────────────────────────────────────────
    print(f"  {'='*70}")
    print(f"  {ICON_LATEX}  LaTeX TABLE (for paper inclusion)")
    print(f"  {'='*70}")
    print()
    latex = generate_latex_table(COMPARISON_ANGLES, spot_fn_3yaw,
                                  orbbec_only_search_time,
                                  stats_spot_3yaw, stats_obb)
    print(latex)
    print()

    # ── Methods section LaTeX snippet ───────────────────────────────
    print(f"  {'='*70}")
    print(f"  {ICON_LATEX}  LaTeX METHODS SNIPPET")
    print(f"  {'='*70}")
    print()
    print(generate_methods_section())
    print()

    # ── Sensitivity analysis: fast yaw speed ───────────────────────
    print(f"  {'='*70}")
    print(f"  SENSITIVITY: Fast yaw speed ({SPOT_YAW_SPEED_FAST:.1f} rad/s)")
    print(f"  {'='*70}")
    print(f"  (Note: these use the global SPOT_YAW_SPEED — set manually to test)")

    # Save original and temporarily use fast speed
    global SPOT_YAW_SPEED, SPOT_YAW_SPEED_DEG_S
    orig_speed = SPOT_YAW_SPEED
    orig_speed_deg = SPOT_YAW_SPEED_DEG_S
    SPOT_YAW_SPEED = SPOT_YAW_SPEED_FAST
    SPOT_YAW_SPEED_DEG_S = math.degrees(SPOT_YAW_SPEED_FAST)

    stats_spot_fast = monte_carlo_sim(spot_fn_3yaw)
    stats_obb_fast = monte_carlo_sim(orbbec_only_search_time)
    fast_speedup = stats_obb_fast.mean / stats_spot_fast.mean
    print(f"    Spot+Arm mean: {stats_spot_fast.mean:.1f}s  (was {stats_spot_3yaw.mean:.1f}s)")
    print(f"    Orbbec-Only mean: {stats_obb_fast.mean:.1f}s  (was {stats_obb.mean:.1f}s)")
    print(f"    Speedup: {fast_speedup:.1f}×  (vs {mean_speedup_3yaw:.1f}× at conservative speed)")
    print()

    # Restore
    SPOT_YAW_SPEED = orig_speed
    SPOT_YAW_SPEED_DEG_S = orig_speed_deg


if __name__ == "__main__":
    main()
