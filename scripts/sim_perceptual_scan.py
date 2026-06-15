#!/usr/bin/env python3
"""Simulate PERCEPTUAL_SCAN poses from wbc_qp_controller.
Generates 6 Cartesian poses around a torso target and prints distances,
so you can see the scan pattern without running the real system."""

import math
import numpy as np

def gen_cartesian_scan_grid(target, nlf_active=False):
    """Replicate _gen_cartesian_scan_grid() from wbc_qp_controller.py"""
    center = target.copy()
    if nlf_active:
        wrist_step = 0.04
        lateral_step = 0.06
        grid_type = 'NLF (tight)'
    else:
        wrist_step = 0.12
        lateral_step = 0.20
        grid_type = 'YOLO (wide)'

    print(f"\n{'='*60}")
    print(f" Grid type: {grid_type}")
    print(f" Center (odom):     [{center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}]")
    print(f" wrist_step:  {wrist_step:.2f}m")
    print(f" lateral_step: {lateral_step:.2f}m")
    print(f"{'='*60}")

    poses = []
    colors = ["🟢", "🟢", "🟢", "🟢", "🔵", "🔵"]
    labels = [
        "wrist LL",
        "wrist LH",
        "wrist HL",
        "wrist HH",
        "lateral -Y",
        "lateral +Y",
    ]

    # Phase 1 — wrist sweep (2×2 = 4 poses)
    i = 0
    for wy in range(2):
        for wz in range(2):
            py = center[1] + (wy - 0.5) * wrist_step
            pz = center[2] + (wz - 0.5) * wrist_step
            dist = math.sqrt((center[0])**2 + py**2 + pz**2)
            poses.append((center[0], py, pz))
            print(f" {colors[i]} Pose {i+1} [{labels[i]:>10}]: "
                  f"x={center[0]:.2f}  y={py:+.2f}  z={pz:+.2f}  dist={dist:.2f}m")
            i += 1

    # Phase 2 — lateral parallax (±Y, 2 poses)
    for j, sign in enumerate([-1.0, 1.0]):
        py = center[1] + sign * lateral_step
        pz = center[2]
        dist = math.sqrt(center[0]**2 + py**2 + pz**2)
        poses.append((center[0], py, pz))
        print(f" {colors[i]} Pose {i+1} [{labels[i]:>10}]: "
              f"x={center[0]:.2f}  y={py:+.2f}  z={pz:+.2f}  dist={dist:.2f}m")
        i += 1

    # ── Arm workspace check ──
    print(f"\n{'─'*60}")
    print(" Arm workspace limits (Z1 from link00):")
    print("   max reach ~0.8m, sweet spot ~0.35-0.50m")
    print(f"   Far target → IK will time out (>3s)")
    for i, (px, py, pz) in enumerate(poses):
        dist = math.sqrt(px**2 + py**2 + pz**2)
        status = "✅ reachable" if dist < 0.8 else "❌ too far (IK timeout)"
        print(f"   Pose {i+1}: dist={dist:.2f}m → {status}")
    print(f"{'─'*60}")

    return poses


if __name__ == "__main__":
    # ── Test targets from real runs ──
    print("\n" + "="*60)
    print(" PERCEPTUAL SCAN SIMULATOR")
    print("="*60)

    # Target from log: torso=[0.94, 1.25, 5.71] - way too far
    # More realistic target ~1-2m away
    targets = [
        np.array([1.0, 0.3, 0.8]),   # close, reachable
        np.array([1.5, 0.5, 1.0]),   # medium
        np.array([2.0, 1.0, 1.5]),   # far, borderline
        np.array([0.94, 1.25, 5.71]), # from real log (broken Z)
    ]

    for t in targets:
        print(f"\n\n▶ Target: [{t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}]")
        gen_cartesian_scan_grid(t, nlf_active=False)
