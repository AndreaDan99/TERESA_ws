# TERESA_ws — Docker for Jetson Orin AGX (L4T R35.x / JetPack 5.x)

---

## Files

| File | Purpose |
|------|---------|
| `Dockerfile.l4t35` | Full Docker image: ROS2 Humble + PyTorch CUDA + RealSense + Orbbec + Z1 arm |
| `docker_entrypoint.sh` | Container entrypoint — sources ROS2, checks GPU & models |
| `README.md` | This file |

---

## 0. Host Requirements (Jetson Orin AGX)

```bash
# A) JetPack 5.1.2+ (L4T R35.4.1)
#    Check with:  cat /etc/nv_tegra_release

# B) Docker with nvidia-container-runtime
#    Setup guide: https://github.com/dusty-nv/jetson-containers/blob/master/docs/setup.md
sudo docker run --rm --runtime=nvidia nvcr.io/nvidia/l4t-jetpack:r35.4.1 nvidia-smi

# C) udev rules for BOTH cameras (MANDATORY, must be on HOST)
# ── Orbbec Femto Bolt ──
sudo cp src/orbbec_camera/orbbec_camera/scripts/99-obsensor-libusb.rules \
        /etc/udev/rules.d/
# ── Intel RealSense D435 ──
#    Usually auto-installed with librealsense2. If not, check:
#    /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# D) Z1 arm network (MANDATORY — configure on HOST, not in Docker)
#    The Z1 arm has static IP 192.168.123.110
#    Set your Jetson's Ethernet interface to:
#      Address:  192.168.123.235
#      Netmask:  255.255.255.0
#      Gateway:  192.168.123.1
```

---

## 1. Intel RealSense D435 (wrist-mounted camera)

| Property | Detail |
|----------|--------|
| **Role** | Z1 wrist-mounted RGBD camera for torso tracking & exposure scanning |
| **ROS2 driver** | `realsense2_camera` — built from `src/realsense-ros/` |
| **SDK** | `librealsense2` v2.56+ |
| **Connection** | USB 3.0 Type-C to Jetson |
| **Topics** | `/camera/camera/color/image_raw`, `/camera/camera/depth/image_rect_raw`, `/camera/camera/aligned_depth_to_color/image_raw` |
| **Config** | RGB+D enabled, pointcloud disabled, aligned depth ON |
| **Frame** | `camera_color_optical_frame` attached to `link06→camera_link` via static TF |

### Docker requirements
```dockerfile
# Already in Dockerfile:
ros-humble-librealsense2
ros-humble-librealsense2-dev
libeigen3-dev
```

### Runtime configuration
```bash
# The project launches it via:
ros2 launch spot_control teresa_core.launch.py
# which internally launches realsense2_camera with:
#   enable_color:=true  enable_depth:=true
#   align_depth.enable:=true
#   pointcloud.enable:=false  (CPU-saving)
#   colorizer.enable:=false
```

### USB pass-through
```bash
# Option 1: --privileged (passes ALL USB devices)
docker run --privileged ...

# Option 2: --device (specific, more secure)
docker run --device=/dev/video0 --device=/dev/video1 --device=/dev/bus/usb ...
```

---

## 2. Unitree Z1 Arm (6-DOF manipulator)

| Property | Detail |
|----------|--------|
| **Role** | 6-DOF robotic arm mounted on Spot, performs ultrasound/exposure scanning |
| **ROS2 packages** | `z1_description`, `z1_bringup`, `z1_hardware_interface`, `z1_moveit`, `z1_examples` |
| **Source** | `src/z1_ros2/` — community package from `github.com/idra-lab/z1_ros2` |
| **SDK** | Unitree `z1_sdk` + `z1_controller` (included in `z1_hardware_interface`) |
| **Connection** | Ethernet — Z1 at `192.168.123.110`, host at `192.168.123.235` |
| **Controllers** | `joint_trajectory_controller` (position, default), `torque_controller` (impedance) |
| **URDF** | `z1_description/urdf/z1.urdf.xacro` with 7 joints (6 arm + gripper) |
| **Frame** | `link00` (base) under `world` frame, `link06` (end-effector), `camera_link` child of `link06` |

### Docker requirements
```dockerfile
# Already in Dockerfile:
ros-humble-ros2-control
ros-humble-ros2-controllers
ros-humble-controller-manager
ros-humble-joint-state-broadcaster
ros-humble-joint-trajectory-controller
ros-humble-effort-controllers
ros-humble-position-controllers
ros-humble-hardware-interface
ros-humble-pluginlib
ros-humble-ament-index-cpp
ros-humble-ament-index-python
ros-humble-moveit
ros-humble-moveit-ros-move-group
ros-humble-moveit-kinematics
ros-humble-moveit-planners
ros-humble-moveit-simple-controller-manager
ros-humble-moveit-configs-utils

# Python:
pinocchio          # IK solver (damped pseudo-inverse Jacobian)
transforms3d       # quaternion_matrix for IK
```

### Network (CRITICAL)
```bash
# The Z1 arm is on a dedicated Ethernet subnet.
# Docker MUST use --net=host to reach 192.168.123.110:
docker run --net=host ...

# Verify connectivity from inside the container:
ping 192.168.123.110
```

### IK Solver
The Z1 arm uses **Pinocchio** for inverse kinematics:
- **Solver**: Damped pseudo-inverse Jacobian (`LOCAL_WORLD_ALIGNED` frame)
- **Trajectory**: Smoothstep quintic interpolation → JTC action
- **Config**: `src/z1_vision/config/z1_ik_jtc_params.yaml`
- **Key params**: `max_joint_vel: 0.8 rad/s`, `ik_tol: 5.0e-3`, `ik_damping: 1.0e-2`

### Multi-controller switching
```bash
# Z1 uses TWO controllers, switched at runtime by safe_controller_switch:
#   1. joint_trajectory_controller (JTC) — position control (homing, approaching)
#   2. torque_controller — effort control (impedance during ultrasound contact)
#
# Services:
#   /safe_switch/to_torque
#   /safe_switch/to_jtc
```

---

## 3. Orbbec Femto Bolt (Spot-mounted camera)

| Property | Detail |
|----------|--------|
| **Role** | Spot-mounted RGBD camera for human skeleton detection & posture classification |
| **ROS2 driver** | `orbbec_camera` v2.6.3 — built from `src/orbbec_camera/` |
| **SDK** | OrbbecSDK v2, from `github.com/orbbec/OrbbecSDK_ROS2` (v2-main branch) |
| **Connection** | USB 3.0 Type-C to Jetson **+ 12V DC power supply** |
| **Topics** | `/orbbec/color/image_raw`, `/orbbec/depth/image_rect_raw` |
| **Firmware** | v1.1.2 (Femto Bolt) |
| **Frame** | `orbbec_color_optical_frame`, child of `orbbec_link` (static TF from `body`) |

### Power (CRITICAL)
```
Orbbec Femto Bolt REQUIRES 12V DC power.
USB-C alone provides insufficient power → camera freezes randomly.
ALWAYS use the external 12V DC barrel jack when running on Spot.
```

### Docker requirements
```dockerfile
# Already in Dockerfile:
libgflags-dev
nlohmann-json3-dev
libgoogle-glog-dev
libssl-dev
libdw-dev
mesa-utils
libgl1
ros-humble-backward-ros
```

### Udev rules (MUST be on HOST, not in container)
```bash
# On the Jetson HOST (before running Docker):
sudo cp src/orbbec_camera/orbbec_camera/scripts/99-obsensor-libusb.rules \
        /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Verify camera detected:
lsusb | grep -i orbbec
# Expected: 2bc5:xxxx Orbbec Femto Bolt
```

### Camera configuration (in project launch files)
```yaml
# Disabled to save CPU + USB bandwidth:
#   - Point cloud / colored point cloud
#   - IR stream
#   - Accelerometer, gyroscope
#   - Auto TF publishing (handled by teresa_core.launch.py)

# Enabled:
#   - RGB:   1280×720 @ 15fps  (MJPG compression)
#   - Depth: 1024×1024 @ 15fps (Y16 format)
#   - Depth registration aligned to RGB
#   - Compression: MJPG for RGB
```

### Launch
```bash
# The Orbbec is launched as part of teresa_core.launch.py:
ros2 launch spot_control teresa_core.launch.py
# This internally launches the femto_bolt.launch.py from orbbec_camera package
```

---

## 4. Full Dependency Map

```
┌──────────────────────────────────────────────────────────────────┐
│                    teresa_jetson:l4t-r35.4.1                     │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│  FROM dustynv/ros:humble-pytorch-l4t-r35.4.1                     │
│  ├── Ubuntu 20.04                                                │
│  ├── ROS2 Humble Desktop (compiled from source)                  │
│  ├── PyTorch 2.0 + torchvision (Jetson CUDA build)               │
│  ├── CUDA 11.4 + cuDNN + TensorRT                               │
│  └── Python 3.8                                                  │
│                                                                   │
│  + apt install:                                                  │
│  ├── [RealSense] librealsense2, librealsense2-dev, eigen3       │
│  ├── [Orbbec]    libgflags-dev, nlohmann-json3-dev, glog, ssl  │
│  ├── [Z1 arm]    ros2_control, ros2_controllers, moveit,        │
│  │               controller_manager, joint_trajectory_controller│
│  ├── [Cameras]   cv_bridge, image_transport, camera_info_manager│
│  └── [Shared]    tf2_*, xacro, rosbridge_suite, visualization   │
│                                                                   │
│  + pip install:                                                  │
│  ├── ultralytics          (YOLO11n-pose)                        │
│  ├── pinocchio            (Z1 IK solver)                        │
│  ├── transforms3d         (quaternion math for IK)              │
│  └── numpy, scipy, matplotlib, opencv-python-headless           │
│                                                                   │
│  + src/ (colcon build):                                         │
│  ├── teresa_utils/        Python (ament_python)                 │
│  ├── z1_vision/           Python (ament_python)                 │
│  ├── spot_control/        Python (ament_python)                 │
│  ├── spot_perception/     Python (ament_python)                 │
│  ├── teresa_demo/         Python (ament_python)                 │
│  ├── z1_ros2/             C++/CMake (ament_cmake)               │
│  ├── realsense-ros/       C++/CMake (ament_cmake)               │
│  └── orbbec_camera/       C++/CMake (ament_cmake)               │
│                                                                   │
│  + Models:                                                       │
│  ├── yolo11n-pose.pt              (~5 MB, auto-download)        │
│  ├── nlf_s_multi.torchscript      (~240 MB, wget from GitHub)   │
│  └── GroundingDINO                (~1.8 GB, HF cache at /work/hf_cache) │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
```

---

## 5. Build

```bash
cd /Users/andrea/Documents/GIT_Repositories/TERESA_ws
docker build -f docker/Dockerfile.l4t35 -t teresa_jetson:l4t-r35.4.1 .
```

Estimated build time: **30-60 minutes** (C++ camera drivers + model downloads).

---

## 6. Run

```bash
sudo docker run -it --rm \
    --runtime=nvidia \          # GPU access (CUDA inference)
    --net=host \                # DDS discovery + Z1 arm (192.168.123.110)
    --privileged \              # USB access for RealSense + Orbbec cameras
    -e ROS_DOMAIN_ID=0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    teresa_jetson:l4t-r35.4.1
```

**Additional mount for GroundingDINO** (exposure scanning):
```bash
-v /ssd/andrea_deploy:/work     # HF model cache for GroundingDINO
```
This is already included in `teresa_start.sh`.

Inside the container, the TERESA startup sequence:
```bash
# T1: Core hardware + TF
ros2 launch spot_control teresa_core.launch.py

# T2: Perception (Orbbec + RealSense)
ros2 launch spot_control teresa_perception.launch.py

# T3: Z1 arm control
ros2 launch z1_vision z1_control.launch.py use_impedance:=false

# T4: WBC coordinator + navigator
ros2 launch spot_control wbc.launch.py

# T5: Keyboard controller
ros2 run spot_control wbc_keyboard_node
```

---

## 7. YAML Changes for Jetson GPU

The repo defaults to CPU inference. For Jetson, change to CUDA:

| File | Param | Before | After |
|------|-------|:------:|:-----:|
| `src/z1_vision/config/z1_yolo_torso_params.yaml` | `device` | `cpu` | **`cuda`** |
| `src/spot_perception/config/nlf_params.yaml` | `device` | `cpu` | **`cuda`** |
| `src/z1_vision/config/nlf_torso_params.yaml` | `device` | `cpu` | **`cuda`** |

---

## 9. HuggingFace Model Cache (GroundingDINO)

GroundingDINO (`IDEA-Research/grounding-dino-base`) is a 1.8 GB transformer model used for
zero-shot wound/scar/burn detection during the Exposure Body Scanning phase.

The model files are stored at `/ssd/andrea_deploy/hf_cache/` on the Jetson host.
Inside the teresa_gpu container, this is mounted at `/work/hf_cache/` via:

```bash
-v /ssd/andrea_deploy:/work
```

The container environment variable `HF_HOME=/work/hf_cache` (set in Dockerfile) tells
HuggingFace transformers to load from this directory. The YAML parameter `cache_dir`
in `injury_detector_params.yaml` explicitly points to the same path.

**Model files** (in `/ssd/andrea_deploy/hf_cache/hub/models--IDEA-Research--grounding-dino-base/`):
- `blobs/` — 10 content-addressed files (2× ~890MB weight files + config + tokenizer + processor)
- `snapshots/` — versioned symlinks to current model revision
- `refs/main` — points to current version

**To verify the model is accessible**:
```bash
docker exec teresa_gpu ls /work/hf_cache/hub/models--IDEA-Research--grounding-dino-base/blobs/
```

**To re-download if missing** (requires internet):
```bash
docker exec teresa_gpu python3 -c "
from transformers import AutoModelForZeroShotObjectDetection
AutoModelForZeroShotObjectDetection.from_pretrained('IDEA-Research/grounding-dino-base')
"
```

---

## 10. Troubleshooting

| Problem | Likely Cause | Fix |
|---------|-------------|-----|
| `torch.cuda.is_available() == False` | Missing `--runtime=nvidia` | Add `--runtime=nvidia` to docker run |
| `no kernel image is available` | Wrong PyTorch (x86, not Jetson) | Use base image's PyTorch, DON'T pip install torch |
| Orbbec not detected | Missing udev rules or USB power | Install udev rules on HOST, use 12V DC power |
| RealSense not detected | USB permission | Use `--privileged` or `--device=/dev/bus/usb` |
| Z1 arm unreachable | Network config | `--net=host`, verify `ping 192.168.123.110` |
| `colcon build` OOM kill | Too many parallel workers | Reduce `--parallel-workers 2` |
| DDS topics not visible | ROS_DOMAIN_ID mismatch or missing `--net=host` | Set same `ROS_DOMAIN_ID`, use `--net=host` |
| NLF model download fails | Network timeout on large file | Run `scripts/download_nlf_models.sh` manually |
| Orbbec camera freezes | USB-C power insufficient | Use 12V DC barrel jack (MANDATORY for Femto Bolt) |
