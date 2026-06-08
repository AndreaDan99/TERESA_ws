# Fix SEARCHING Rotation — Remove TF, Use Timed Open-Loop

## TL;DR

> **Quick Summary**: Remove TF dependency from coarse search rotation. Replace P-controlled TF-based rotation with timed open-loop `cmd_vel.angular.z` for 2.1s per 60° step. TF kept for SEMI_LOCKING/LOCKING precision only.
>
> **Deliverables**:
> - `wbc_coordinator.py` — simplified rotation in `_tick_search()`: no TF, just timed angular velocity

## Context

### Problem
TF `odom→body` is unreliable during SEARCHING because Spot's odometry drifts. The current code uses TF for yaw P-control during coarse rotation → stalls when TF drops.

### Solution
Coarse search rotation doesn't need precision. Replace the entire TF-based P-control (lines 960-996) with timed open-loop: publish `cmd_vel.angular.z = search_max_angular_vel` for `search_yaw_increment / search_max_angular_vel` seconds (~2.1s). No TF lookup at all. TF remains for SEMI_LOCKING and LOCKING.

## Work Objectives

### Core Objective
SEARCHING coarse rotation works 100% of the time, regardless of TF state.

### Concrete Changes
- Replace lines 956-996 of `_tick_search()` (the entire "Rotating: P-control" block) with timed open-loop
- Remove `_last_yaw_error` (no longer needed)
- Remove `_get_current_yaw()` lookup during rotation
- Keep `_search_target_yaw` only for logging

### Must NOT Have
- Do NOT change SEMI_LOCKING or LOCKING TF usage
- Do NOT change search sequence generation

## TODOs

- [x] 1. Replace P-control rotation with timed open-loop in `_tick_search()`

  **What to do**: Replace lines 956-996 (from `# ── Rotating: P-control yaw via cmd_vel.angular.z ──` to the end of rotation block) with:
  ```python
  # ── Rotating: timed open-loop (no TF needed) ──
  if self._search_rotating:
      elapsed = (self.get_clock().now() - self._search_rotation_start).nanoseconds / 1e9
      expected = self._search_yaw_increment / self._search_max_angular_vel
      if elapsed >= expected:
          self._pub_cmd_vel.publish(Twist())
          self._search_rotating = False
          self._search_position_start = self.get_clock().now()
          self.get_logger().info(
              f'Search pos {self._search_position_idx+1}: yaw step done '
              f'({elapsed:.1f}s) → dwell {self._search_coarse_dwell:.0f}s')
          return
      t = Twist()
      t.angular.z = self._search_max_angular_vel
      self._pub_cmd_vel.publish(t)
      return
  ```

- [x] 2. Record rotation start time when entering rotating mode

  **What to do**: Find where `_search_rotating = True` is set and add `self._search_rotation_start = self.get_clock().now()` right after it.

- [x] 3. Add `_search_rotation_start` state variable

  **What to do**: Add `self._search_rotation_start: rclpy.time.Time | None = None` in `__init__` near other search state variables (~line 299).

- [x] 4. Clean up unused variables

  **What to do**: Remove `_last_yaw_error` (line 299) — no longer needed. Remove `_search_target_yaw` computation (lines 948-950) or keep only for logging.

## Commit Strategy
```
fix(wbc): replace TF-based coarse rotation with timed open-loop

- SEARCHING coarse yaw steps now use timed cmd_vel.angular.z (2.1s/step)
- No TF odom→body dependency for rotation — works even with odometry loss
- TF still used for SEMI_LOCKING/LOCKING precision alignment
```

- [ ] 4. Verify TF-based rotation still works when TF is available

  **What to do**: The existing code at lines 974-996 must remain unchanged for the TF-available path.

## Commit Strategy

```
fix(wbc): use timed open-loop rotation when TF odom→body unavailable

- Replace stale _last_yaw_error fallback with time-based rotation
- Rotate at search_max_angular_vel for search_yaw_increment/max_vel seconds
- TF-based P-control rotation unchanged when TF is available
```
