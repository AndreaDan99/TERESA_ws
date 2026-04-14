# spot_navigation — Design Spec

**Date:** 2026-04-14
**Status:** Approved

## Summary

New ROS2 package `spot_navigation` in TERESA_ws. Runs on Jetson (local), communicates with SpotCore via ROS2 DDS. Subscribes to approach points from `spot_perception`, navigates Spot to the goal via `cmd_vel` on manual keyboard trigger. Spot handles obstacle avoidance internally.

---

## Architecture

```
TERESA_ws/src/spot_navigation/
├── spot_navigation/
│   ├── __init__.py
│   └── spot_goal_navigator.py   ← single node
├── launch/
│   └── spot_navigation.launch.py
├── config/
│   └── spot_nav_params.yaml
├── package.xml
└── setup.py
```

---

## Data Flow

```
laying_human_detector (spot_perception)
  └─► /laying_human/approach_point (PoseStamped, camera_color_optical_frame)
        └─► spot_goal_navigator
              ├─► store latest goal (always updated, no auto-start)
              ├─► stdin thread: wait for 's' + Enter
              ├─► on 's': TF lookup camera_color_optical_frame → body
              ├─► Phase 1 ROTATING: publish angular.z on /cmd_vel (linear.x=0)
              ├─► Phase 2 DRIVING:  publish linear.x on /cmd_vel (+ small angular.z correction)
              └─► STOPPED: publish zero Twist, return to IDLE
```

SpotCore runs `spot_driver` which subscribes to `cmd_vel` (relative topic, resolved by namespace).

---

## Node: `spot_goal_navigator`

### Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `/laying_human/approach_point` | `geometry_msgs/PoseStamped` |
| Pub | `cmd_vel_topic` (param) | `geometry_msgs/Twist` |

### Parameters (`spot_nav_params.yaml`)

```yaml
spot_goal_navigator:
  ros__parameters:
    cmd_vel_topic: "/cmd_vel"     # set to "/my_spot/cmd_vel" if spot_name="my_spot"
    goal_tolerance: 0.3           # meters — stop when dist < this
    angular_speed_max: 0.5        # rad/s
    linear_speed_max: 0.4         # m/s
    angle_threshold: 0.15         # rad — end of ROTATING phase
    robot_frame: "body"           # Spot body frame
    update_rate: 10.0             # Hz — control loop rate
```

### State Machine

```
IDLE
  │  approach_point received → stored (no motion)
  │  user presses 's' + Enter → if no goal: warn, stay IDLE
  ▼
ROTATING
  │  angular.z = clip(Kp_ang * angle_error, -angular_speed_max, angular_speed_max)
  │  linear.x = 0
  │  |angle_error| < angle_threshold
  ▼
DRIVING
  │  linear.x = clip(Kp_lin * dist, 0, linear_speed_max)
  │  angular.z = clip(Kp_ang * angle_error, -angular_speed_max/2, angular_speed_max/2)
  │  dist < goal_tolerance
  ▼
STOPPED
  │  publish zero Twist
  │  log "Goal reached"
  ▼
IDLE  ← wait for next 's'
```

**Goal preemption:** new approach_point received in any active state → stored. New 's' press always resets to ROTATING with latest goal.

### TF

- Input goal frame: `camera_color_optical_frame`
- Transform to `body` frame at navigation start
- Use `tf2_ros.Buffer.transform()` with timeout

---

## Error Handling

| Condition | Behavior |
|-----------|----------|
| No goal when 's' pressed | Warn, stay IDLE |
| TF lookup fails | Warn, stay IDLE, do not publish cmd_vel |
| Goal received but not started | Silently update stored goal |
| Node shutdown mid-navigation | Publish zero Twist in destructor |

---

## Launch

```bash
# On Jetson — after spot_perception is running
ros2 launch spot_navigation spot_navigation.launch.py

# If spot_name="my_spot" on SpotCore:
ros2 launch spot_navigation spot_navigation.launch.py cmd_vel_topic:=/my_spot/cmd_vel
```

Prerequisite: `spot_driver` running on SpotCore with same `ROS_DOMAIN_ID`. Spot already standing.

---

## Out of Scope (future)

- Spot startup (claim/power_on/stand) — handled manually
- Obstacle avoidance — handled by Spot's internal systems
- Path planning — direct two-phase controller only
- Return-to-start — handled manually via joystick
