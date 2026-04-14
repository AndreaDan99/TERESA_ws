# spot_navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** New ROS2 ament_python package `spot_navigation` that subscribes to `/laying_human/approach_point`, waits for keyboard `s` + Enter, then navigates Spot to the goal via `cmd_vel` using a two-phase (rotate → drive) P-controller.

**Architecture:** Single node `SpotGoalNavigatorNode` with a pure-function `compute_cmd_vel` for testable velocity logic and a `NavState` enum for state machine. Keyboard input runs on a daemon thread reading stdin. TF lookup transforms the goal from `camera_color_optical_frame` to `body` at navigation start.

**Tech Stack:** ROS2 Humble, rclpy, tf2_ros, geometry_msgs, ament_python, pytest

---

## File Map

| File | Action | Responsibility |
|------|--------|----------------|
| `src/spot_navigation/package.xml` | Create | ROS2 package manifest |
| `src/spot_navigation/setup.py` | Create | ament_python entry points |
| `src/spot_navigation/spot_navigation/__init__.py` | Create | Package init |
| `src/spot_navigation/spot_navigation/spot_goal_navigator.py` | Create | Node + NavState + compute_cmd_vel |
| `src/spot_navigation/config/spot_nav_params.yaml` | Create | Tunable parameters |
| `src/spot_navigation/launch/spot_navigation.launch.py` | Create | Launch file |
| `src/spot_navigation/test/test_navigation_logic.py` | Create | Unit tests for pure logic |

---

## Task 1: Package scaffold

**Files:**
- Create: `src/spot_navigation/package.xml`
- Create: `src/spot_navigation/setup.py`
- Create: `src/spot_navigation/spot_navigation/__init__.py`

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p src/spot_navigation/spot_navigation
mkdir -p src/spot_navigation/config
mkdir -p src/spot_navigation/launch
mkdir -p src/spot_navigation/test
touch src/spot_navigation/spot_navigation/__init__.py
touch src/spot_navigation/test/__init__.py
echo "spot_navigation" > src/spot_navigation/resource/spot_navigation
mkdir -p src/spot_navigation/resource
echo "spot_navigation" > src/spot_navigation/resource/spot_navigation
```

- [ ] **Step 2: Write `package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>spot_navigation</name>
  <version>0.0.0</version>
  <description>Keyboard-triggered goal navigation for Spot via cmd_vel</description>
  <maintainer email="andrea.dantona@unife.it">andrea</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: Write `setup.py`**

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrea',
    maintainer_email='andrea.dantona@unife.it',
    description='Keyboard-triggered goal navigation for Spot via cmd_vel',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spot_goal_navigator = spot_navigation.spot_goal_navigator:main',
        ],
    },
)
```

- [ ] **Step 4: Verify package builds**

```bash
cd /Users/andrea/Documents/GIT_Repositories/TERESA_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select spot_navigation
```

Expected: build succeeds (no node yet, just scaffold).

- [ ] **Step 5: Commit**

```bash
git add src/spot_navigation/
git commit -m "feat(spot_navigation): add package scaffold"
```

---

## Task 2: Navigation logic (pure functions — TDD)

**Files:**
- Create: `src/spot_navigation/test/test_navigation_logic.py`
- Create: `src/spot_navigation/spot_navigation/spot_goal_navigator.py` (logic only, no ROS2 yet)

- [ ] **Step 1: Write failing tests**

Create `src/spot_navigation/test/test_navigation_logic.py`:

```python
import math
import pytest
from spot_navigation.spot_goal_navigator import NavState, compute_cmd_vel


# ── Fixtures ────────────────────────────────────────────────────────────────

class Params:
    angular_speed_max = 0.5
    linear_speed_max  = 0.4
    angle_threshold   = 0.15
    goal_tolerance    = 0.3
    kp_ang            = 1.0
    kp_lin            = 0.5


P = Params()


# ── ROTATING state ──────────────────────────────────────────────────────────

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


# ── DRIVING state ───────────────────────────────────────────────────────────

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


# ── IDLE / STOPPED states ───────────────────────────────────────────────────

def test_idle_returns_zero_twist():
    twist = compute_cmd_vel(dx=1.0, dy=0.5, state=NavState.IDLE, params=P)
    assert twist.linear.x == 0.0
    assert twist.angular.z == 0.0

def test_stopped_returns_zero_twist():
    twist = compute_cmd_vel(dx=1.0, dy=0.5, state=NavState.STOPPED, params=P)
    assert twist.linear.x == 0.0
    assert twist.angular.z == 0.0
```

- [ ] **Step 2: Run tests — verify they FAIL**

```bash
cd /Users/andrea/Documents/GIT_Repositories/TERESA_ws
source install/setup.bash
python3 -m pytest src/spot_navigation/test/test_navigation_logic.py -v
```

Expected: `ImportError` — `spot_goal_navigator` not yet defined.

- [ ] **Step 3: Implement `NavState` and `compute_cmd_vel` in `spot_goal_navigator.py`**

Create `src/spot_navigation/spot_navigation/spot_goal_navigator.py`:

```python
#!/usr/bin/env python3
import math
import threading
import sys
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support


class NavState(Enum):
    IDLE     = auto()
    ROTATING = auto()
    DRIVING  = auto()
    STOPPED  = auto()


def compute_cmd_vel(dx: float, dy: float, state: 'NavState', params) -> Twist:
    """Pure function: compute Twist from goal offset in body frame.

    Args:
        dx: goal x in body frame (forward = positive)
        dy: goal y in body frame (left = positive)
        state: current NavState
        params: object with angular_speed_max, linear_speed_max, kp_ang, kp_lin

    Returns:
        geometry_msgs/Twist
    """
    twist = Twist()

    if state not in (NavState.ROTATING, NavState.DRIVING):
        return twist  # zero twist for IDLE / STOPPED

    angle_to_goal = math.atan2(dy, dx)
    dist          = math.hypot(dx, dy)

    if state == NavState.ROTATING:
        raw_ang = params.kp_ang * angle_to_goal
        twist.angular.z = float(max(-params.angular_speed_max,
                                    min(params.angular_speed_max, raw_ang)))

    elif state == NavState.DRIVING:
        raw_lin = params.kp_lin * dist
        twist.linear.x = float(max(0.0, min(params.linear_speed_max, raw_lin)))

        raw_ang = params.kp_ang * angle_to_goal
        half    = params.angular_speed_max / 2.0
        twist.angular.z = float(max(-half, min(half, raw_ang)))

    return twist


# ── ROS2 Node ────────────────────────────────────────────────────────────────
# (implemented in Task 3)
```

- [ ] **Step 4: Run tests — verify they PASS**

```bash
python3 -m pytest src/spot_navigation/test/test_navigation_logic.py -v
```

Expected: all 10 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/spot_navigation/spot_navigation/spot_goal_navigator.py \
        src/spot_navigation/test/test_navigation_logic.py
git commit -m "feat(spot_navigation): add NavState + compute_cmd_vel with tests"
```

---

## Task 3: ROS2 node

**Files:**
- Modify: `src/spot_navigation/spot_navigation/spot_goal_navigator.py` (add Node class + main)

- [ ] **Step 1: Append `SpotGoalNavigatorNode` class and `main()` to `spot_goal_navigator.py`**

Append after the existing `compute_cmd_vel` function:

```python
class _Params:
    """Holds node parameters as plain attributes for use with compute_cmd_vel."""
    def __init__(self, node: Node):
        self.cmd_vel_topic     = node.get_parameter('cmd_vel_topic').value
        self.goal_tolerance    = float(node.get_parameter('goal_tolerance').value)
        self.angular_speed_max = float(node.get_parameter('angular_speed_max').value)
        self.linear_speed_max  = float(node.get_parameter('linear_speed_max').value)
        self.angle_threshold   = float(node.get_parameter('angle_threshold').value)
        self.robot_frame       = node.get_parameter('robot_frame').value
        self.odom_frame        = node.get_parameter('odom_frame').value
        self.update_rate       = float(node.get_parameter('update_rate').value)
        self.kp_ang            = 1.0
        self.kp_lin            = 0.5


class SpotGoalNavigatorNode(Node):

    def __init__(self):
        super().__init__('spot_goal_navigator')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('cmd_vel_topic',    '/cmd_vel')
        self.declare_parameter('goal_tolerance',    0.3)
        self.declare_parameter('angular_speed_max', 0.5)
        self.declare_parameter('linear_speed_max',  0.4)
        self.declare_parameter('angle_threshold',   0.15)
        self.declare_parameter('robot_frame',       'body')
        self.declare_parameter('odom_frame',        'odom')
        self.declare_parameter('update_rate',       10.0)

        self._p = _Params(self)

        # ── TF ──────────────────────────────────────────────────────────────
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Sub / Pub ────────────────────────────────────────────────────────
        self._goal_sub = self.create_subscription(
            PoseStamped,
            '/laying_human/approach_point',
            self._cb_goal,
            10,
        )
        self._cmd_pub = self.create_publisher(Twist, self._p.cmd_vel_topic, 10)

        # ── State ─────────────────────────────────────────────────────────────
        # _latest_goal: raw approach point in camera frame (updated by sub)
        # _goal_odom:   goal transformed to odom frame at press of 's' (world-fixed)
        self._state: NavState                = NavState.IDLE
        self._latest_goal: PoseStamped | None = None
        self._goal_odom:   PoseStamped | None = None
        self._lock = threading.Lock()

        # ── Control loop timer ────────────────────────────────────────────────
        period = 1.0 / self._p.update_rate
        self._timer = self.create_timer(period, self._control_loop)

        # ── Keyboard thread ───────────────────────────────────────────────────
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        self.get_logger().info(
            f'SpotGoalNavigator ready.\n'
            f'  cmd_vel → {self._p.cmd_vel_topic}\n'
            f'  robot_frame: {self._p.robot_frame}\n'
            f'Press "s" + Enter to start navigation to latest approach point.'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_goal(self, msg: PoseStamped) -> None:
        """Store latest approach point (raw camera frame)."""
        with self._lock:
            self._latest_goal = msg

    def _keyboard_loop(self) -> None:
        """Blocking stdin reader — runs on daemon thread."""
        while rclpy.ok():
            try:
                line = sys.stdin.readline().strip()
            except EOFError:
                break
            if line == 's':
                self._on_start_key()

    def _on_start_key(self) -> None:
        with self._lock:
            goal_raw = self._latest_goal

        if goal_raw is None:
            self.get_logger().warn('No approach point received yet — cannot start.')
            return

        # Transform to odom (world-fixed) frame at press time.
        # Use time=0 (latest available TF) to avoid stale-timestamp issues.
        goal_stamped = PoseStamped()
        goal_stamped.header.frame_id = goal_raw.header.frame_id
        goal_stamped.header.stamp    = rclpy.time.Time().to_msg()
        goal_stamped.pose            = goal_raw.pose

        try:
            goal_odom = self._tf_buffer.transform(
                goal_stamped,
                self._p.odom_frame,
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e} — navigation not started.')
            return

        with self._lock:
            self._goal_odom = goal_odom
            self._state     = NavState.ROTATING

        self.get_logger().info(
            f'Navigation started → '
            f'({goal_odom.pose.position.x:.2f}, {goal_odom.pose.position.y:.2f}) [odom]'
        )

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        with self._lock:
            state     = self._state
            goal_odom = self._goal_odom

        if state == NavState.IDLE or goal_odom is None:
            return

        # Re-transform goal from odom → body each tick so dx/dy reflect
        # current robot position (body frame moves with Spot).
        goal_body_stamped = PoseStamped()
        goal_body_stamped.header.frame_id = self._p.odom_frame
        goal_body_stamped.header.stamp    = rclpy.time.Time().to_msg()
        goal_body_stamped.pose            = goal_odom.pose

        try:
            goal_body = self._tf_buffer.transform(
                goal_body_stamped,
                self._p.robot_frame,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException as e:
            self.get_logger().warn(f'TF error in control loop: {e}', throttle_duration_sec=2.0)
            return

        dx = goal_body.pose.position.x
        dy = goal_body.pose.position.y
        dist          = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)

        # ── State transitions ─────────────────────────────────────────────────
        if state == NavState.ROTATING:
            if abs(angle_to_goal) < self._p.angle_threshold:
                with self._lock:
                    self._state = NavState.DRIVING
                self.get_logger().info('Phase 2: DRIVING')
                state = NavState.DRIVING

        if state == NavState.DRIVING:
            if dist < self._p.goal_tolerance:
                with self._lock:
                    self._state = NavState.STOPPED
                self._cmd_pub.publish(Twist())
                self.get_logger().info('Goal reached — STOPPED. Press "s" for next goal.')
                return

        if state == NavState.STOPPED:
            with self._lock:
                self._state = NavState.IDLE
            return

        # ── Publish velocity ──────────────────────────────────────────────────
        twist = compute_cmd_vel(dx, dy, state, self._p)
        self._cmd_pub.publish(twist)

    def destroy_node(self) -> None:
        """Publish zero Twist on shutdown."""
        self._cmd_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpotGoalNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: Re-run unit tests to confirm no regression**

```bash
python3 -m pytest src/spot_navigation/test/test_navigation_logic.py -v
```

Expected: all 10 tests still PASS.

- [ ] **Step 3: Build package**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select spot_navigation
source install/setup.bash
```

Expected: build succeeds, executable `spot_goal_navigator` registered.

- [ ] **Step 4: Commit**

```bash
git add src/spot_navigation/spot_navigation/spot_goal_navigator.py
git commit -m "feat(spot_navigation): add SpotGoalNavigatorNode with two-phase controller"
```

---

## Task 4: Config + Launch

**Files:**
- Create: `src/spot_navigation/config/spot_nav_params.yaml`
- Create: `src/spot_navigation/launch/spot_navigation.launch.py`

- [ ] **Step 1: Write `spot_nav_params.yaml`**

```yaml
spot_goal_navigator:
  ros__parameters:
    # Topic that spot_driver subscribes to.
    # Use "/my_spot/cmd_vel" if spot_name="my_spot" in spot_driver config.
    cmd_vel_topic: "/cmd_vel"

    # Stop when closer than this to goal (metres)
    goal_tolerance: 0.3

    # Phase 1 ends when |angle_to_goal| < this (radians)
    angle_threshold: 0.15

    # Velocity limits
    angular_speed_max: 0.5    # rad/s
    linear_speed_max: 0.4     # m/s

    # Spot body TF frame name (published by spot_driver, moves with robot)
    robot_frame: "body"

    # World-fixed frame (goal stored here between keypress and arrival)
    odom_frame: "odom"

    # Control loop rate
    update_rate: 10.0    # Hz
```

- [ ] **Step 2: Write `spot_navigation.launch.py`**

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='cmd_vel topic — match spot_driver namespace. '
                    'Use /my_spot/cmd_vel if spot_name="my_spot".',
    )

    params_file = PathJoinSubstitution([
        FindPackageShare('spot_navigation'),
        'config',
        'spot_nav_params.yaml',
    ])

    navigator_node = Node(
        package='spot_navigation',
        executable='spot_goal_navigator',
        name='spot_goal_navigator',
        output='screen',
        parameters=[
            params_file,
            {'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic')},
        ],
    )

    return LaunchDescription([
        cmd_vel_topic_arg,
        navigator_node,
    ])
```

- [ ] **Step 3: Build and smoke-test launch**

```bash
colcon build --packages-select spot_navigation
source install/setup.bash
# In a separate terminal, verify the node starts and prints the ready message:
ros2 launch spot_navigation spot_navigation.launch.py
```

Expected output:
```
[spot_goal_navigator]: SpotGoalNavigator ready.
[spot_goal_navigator]:   cmd_vel → /cmd_vel
[spot_goal_navigator]:   robot_frame: body
[spot_goal_navigator]: Press "s" + Enter to start navigation to latest approach point.
```

- [ ] **Step 4: Commit**

```bash
git add src/spot_navigation/config/spot_nav_params.yaml \
        src/spot_navigation/launch/spot_navigation.launch.py
git commit -m "feat(spot_navigation): add config and launch file"
```

---

## Task 5: Integration smoke-test (no hardware)

Verify the node responds correctly to synthetic topics before connecting Spot.

- [ ] **Step 1: Start the node**

Terminal 1:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch spot_navigation spot_navigation.launch.py
```

- [ ] **Step 2: Publish a static TF (body frame)**

Terminal 2:
```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 body camera_color_optical_frame
```

- [ ] **Step 3: Publish a fake approach point**

Terminal 3:
```bash
ros2 topic pub --once /laying_human/approach_point geometry_msgs/PoseStamped \
  '{header: {frame_id: "camera_color_optical_frame"}, pose: {position: {x: 2.0, y: 0.3, z: 0.0}, orientation: {w: 1.0}}}'
```

Expected: node logs `Stored new approach point` (no motion yet).

- [ ] **Step 4: Press `s` in Terminal 1**

Type `s` + Enter in the terminal running the node.

Expected:
```
Navigation started → (2.00, 0.30) [body]
Phase 2: DRIVING
```

- [ ] **Step 5: Verify cmd_vel published**

Terminal 4:
```bash
ros2 topic echo /cmd_vel --once
```

Expected: non-zero Twist with `linear.x > 0` or `angular.z != 0`.

- [ ] **Step 6: Commit**

```bash
git add .
git commit -m "feat(spot_navigation): complete — smoke-tested without hardware"
```

---

## Usage Summary

```bash
# SpotCore
ros2 launch spot_driver spot_driver.launch.py \
  config_file:=/path/to/my_spot.yaml

# Jetson — Terminal 1: perception
ros2 launch spot_perception spot_perception.launch.py test_mode:=false

# Jetson — Terminal 2: navigation (run in foreground, keyboard input required)
ros2 launch spot_navigation spot_navigation.launch.py
# Press 's' + Enter to navigate Spot to latest detected lying human
```

If `spot_name: "my_spot"` in spot_driver config:
```bash
ros2 launch spot_navigation spot_navigation.launch.py cmd_vel_topic:=/my_spot/cmd_vel
```
