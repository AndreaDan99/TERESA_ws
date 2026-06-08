# Sequential SEARCHING — Rotation then Arm

## Changes

### 1. `wbc_coordinator.py` — Sequential rotation steps

Replace single `search_yaw_increment` with alternating list: [30, -60, 90, -120, 150, -180] degrees.
Each step: rotate → wait for arm to finish 3 poses via `/ik_done` → next step.

Remove dwell timer — dwell = wait for arm completion.

### 2. `wbc_qp_controller.py` — Hardcoded manual poses

Replace computed offsets with:
```python
SEARCH_POSES = [
    ([0.144, -0.005,  0.530], [0.0182,  0.1521, -0.0217, 0.9880]),
    ([0.067, -0.070,  0.540], [0.0906,  0.1890, -0.3976, 0.8932]),
    ([0.057,  0.079,  0.538], [-0.0888, 0.1933,  0.4310, 0.8769]),
]
```

### 3. `z1_ik_jtc_params.yaml` — Speed up arm

max_joint_vel: 0.2 → 0.4 rad/s (back to faster)

### 4. `wbc_params.yaml` — Yaw angles as list

search_yaw_increment → search_yaw_angles: [30, -60, 90, -120, 150, -180] degrees
