# TERESA_ws — Docker for Jetson Orin AGX (L4T R35.x)

## Files

| File | Purpose |
|------|---------|
| `Dockerfile.l4t35` | Docker image for L4T R35.x (JetPack 5.1.2) |
| `docker_entrypoint.sh` | Container entrypoint — sources ROS2 + checks GPU/models |

## Prerequisites (on Jetson host)

```bash
# 1. JetPack 5.1.2+ (L4T R35.4.1)
# 2. Docker with nvidia-container-runtime
#    See: https://github.com/dusty-nv/jetson-containers/blob/master/docs/setup.md

# 3. Verify Docker GPU access
sudo docker run --rm --runtime=nvidia nvcr.io/nvidia/l4t-jetpack:r35.4.1 nvidia-smi

# 4. Install Orbbec udev rules ON THE HOST (not in container)
sudo cp src/orbbec_camera/orbbec_camera/scripts/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Build

```bash
cd /path/to/TERESA_ws
docker build -f docker/Dockerfile.l4t35 -t teresa_jetson:l4t-r35.4.1 .
```

Build time: ~20-40 min (mostly `colcon build` and model downloads).

## Run

```bash
sudo docker run -it --rm \
    --runtime=nvidia \
    --net=host \
    --privileged \
    -e ROS_DOMAIN_ID=0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    teresa_jetson:l4t-r35.4.1
```

## Key differences from x86 desktop

| Aspect | x86 Laptop | Jetson Orin AGX |
|--------|:----------:|:---------------:|
| Architecture | x86_64 | **ARM64 (aarch64)** |
| ROS2 distro | Jazzy | **Humble** |
| ROS2 install | apt binary | Compiled from source |
| PyTorch | pip standard | **Jetson build (pypi.dusty-nv.com)** |
| GPU | ❌ (CPU-only) | ✅ **CUDA 11.4, 2048 cores** |
| YOLO device | `cpu` | **`cuda`** |
| NLF device | `cpu` | **`cuda`** |
| Base image | `ubuntu:24.04` | `dustynv/ros:humble-pytorch-l4t-r35.4.1` |
| Docker runtime | default | **`--runtime=nvidia`** |
| Network | bridge | **`--net=host`** (DDS discovery) |

## YAML config changes needed

The repo YAML files default to `device: "cpu"`. For Jetson GPU inference, change:

```yaml
# src/z1_vision/config/z1_yolo_torso_params.yaml
device: "cuda"   # was "cpu"

# src/spot_perception/config/nlf_params.yaml
device: "cuda"   # was "cpu"

# src/z1_vision/config/nlf_torso_params.yaml
device: "cuda"   # was "cpu"
```

## Notes

- **Orin AGX has 32/64 GB RAM** — `--parallel-workers 4` prevents OOM during build
- **Camera USB devices** must be passed with `--privileged` or `--device=/dev/bus/usb`
- **DDS discovery** requires `--net=host` to reach Spot robot and Z1 arm on the network
- **rosdep may warn** about missing keys — this is normal when building from source on L4T
- The `dustynv/ros:humble-pytorch-l4t-r35.4.1` base image is ~10 GB compressed
