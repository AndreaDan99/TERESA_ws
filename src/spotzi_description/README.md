# SpotZi Simulation

Combined Boston Dynamics Spot + Unitree Z1 arm simulation environment.

## Structure

```
SpotZi_Simulation/
├── spotzi_description/        # Combined URDF + launch + RViz config
│   ├── urdf/
│   │   ├── spotzi.urdf.xacro  # Top-level combined URDF
│   │   ├── spot_macro.xacro   # Spot body + 4 legs (adapted from spot_description)
│   │   ├── z1_macro.xacro     # Z1 6-DOF arm (adapted from z1_description)
│   │   ├── const.xacro        # Z1 inertial constants
│   │   └── accessories.urdf.xacro  # Spot accessories
│   ├── meshes/                # Symlinks to original mesh repos
│   │   ├── base/              # Spot body & leg meshes
│   │   ├── spot_arm/           # Spot arm meshes (unused)
│   │   └── z1/                # Z1 arm meshes (visual + collision)
│   ├── launch/
│   │   └── display.launch.py  # RViz visualization
│   └── rviz/
│       └── spotzi.rviz
├── launch/                    # Future: full sim launch files
├── config/                    # Future: simulation configs
└── COLCON_IGNORE              # Not a full workspace, just the description package
```

## Quick Start

### Prerequisites

```bash
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-rviz2
```

### Build

```bash
cd SpotZi_Simulation
colcon build --packages-select spotzi_description
source install/setup.bash
```

### Launch

```bash
# Full Spot + Z1 with gripper
ros2 launch spotzi_description display.launch.py

# Without gripper
ros2 launch spotzi_description display.launch.py with_gripper:=false

# Custom Z1 mount position
ros2 launch spotzi_description display.launch.py z1_mount_x:=0.25 z1_mount_z:=0.18

# No RViz, just URDF check
ros2 launch spotzi_description display.launch.py rviz:=false gui:=false
```

### Validate URDF

```bash
xacro spotzi_description/urdf/spotzi.urdf.xacro > /tmp/spotzi.urdf
check_urdf /tmp/spotzi.urdf
```

## Kinematic Tree

```
body (Spot root)
├── base_link (inertial reference)
├── front_rail / rear_rail
├── front_left_hip → upper_leg → lower_leg
├── front_right_hip → upper_leg → lower_leg
├── rear_left_hip → upper_leg → lower_leg
├── rear_right_hip → upper_leg → lower_leg
└── world (Z1 mount point, fixed joint at 0.20, 0, 0.20)
    └── link00 → link01 → link02 → link03 → link04 → link05 → link06
        └── gripperStator → gripperMover (optional)
```

## Mesh Sources

Meshes are symlinked from:
- `TERESA_ws/src/z1_ros2/z1_description/meshes/` → `meshes/z1/`
- `ros2_spot_ws/src/spot_description/spot_description/meshes/base/` → `meshes/base/`
- `ros2_spot_ws/src/spot_description/spot_description/meshes/arm/` → `meshes/spot_arm/`

## Future Work

- [ ] Gazebo simulation with ros2_control mock
- [ ] Spot mock node (publishes odom→body TF)
- [ ] Full TERESA pipeline (z1_vision + spot_control + WBC)
- [ ] Unity export (URDF → FBX conversion)
