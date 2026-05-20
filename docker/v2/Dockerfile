# ─────────────────────────────────────────────────────────────────────────────
# TERESA Jetson Docker — v2
# Base: dustynv ROS2 Humble + CUDA per JetPack 6.x (L4T r36 / Ubuntu 22.04)
#
# Su r36 (Jammy) i pacchetti ros-humble-* sono disponibili via apt.
# Tutto (pinocchio, ros2-control, xacro, rviz2...) installato da apt.
#
# Build:
#   docker build -t teresa_ws:v2 .
#
# Run:
#   ./run.sh
#
# VERIFICA la versione L4T sulla Jetson prima di buildare:
#   cat /etc/nv_tegra_release
# ─────────────────────────────────────────────────────────────────────────────
FROM dustynv/ros:humble-ros-base-l4t-r36.2.0

# ─── Fix ROS2 GPG key (expired) ─────────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# ─── System + ROS2 deps (apt, funziona su Jammy) ────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential cmake git \
    python3-pip python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    # ROS2 control stack (NON in ros_base)
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-trajectory-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-effort-controllers \
    ros-humble-position-controllers \
    # Extra ROS2 (NON in ros_base)
    ros-humble-tf-transformations \
    ros-humble-rviz2 \
    ros-humble-pinocchio \
    # NOTE: cv_bridge, image_transport, tf2_ros, xacro, robot_state_publisher,
    # message_filters, visualization_msgs sono GIÀ nel base image (source-built).
    # Orbbec camera build deps
    libgflags-dev \
    nlohmann-json3-dev \
    libgoogle-glog-dev \
    libssl-dev \
    # Eigen / fmt (z1 hardware interface)
    libeigen3-dev \
    libfmt-dev \
    # USB / udev (cameras + Z1 arm)
    udev \
    libusb-1.0-0-dev \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# ─── Intel RealSense SDK (arm64, RSUSB backend) ─────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    libssl-dev libusb-1.0-0-dev libudev-dev \
    pkg-config libgtk-3-dev \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch v2.57.7 \
    https://github.com/IntelRealSense/librealsense.git /tmp/librealsense && \
    mkdir -p /tmp/librealsense/build && cd /tmp/librealsense/build && \
    cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_GRAPHICAL_EXAMPLES=OFF \
      -DFORCE_RSUSB_BACKEND=ON && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/librealsense

# ─── Orbbec SDK per Femto Bolt (ARM64) ──────────────────────────────────────
ARG ORBBEC_SDK_VERSION=v1.10.27
RUN apt-get update && apt-get install -y --no-install-recommends unzip \
    && rm -rf /var/lib/apt/lists/* \
    && wget -q \
    "https://github.com/orbbec/OrbbecSDK/releases/download/${ORBBEC_SDK_VERSION}/OrbbecSDK_C_C%2B%2B_${ORBBEC_SDK_VERSION}_20250925_0549823_linux_arm64_release.zip" \
    -O /tmp/orbbec_sdk.zip \
    && mkdir -p /opt/orbbec_sdk \
    && unzip -q /tmp/orbbec_sdk.zip -d /opt/orbbec_sdk \
    && find /opt/orbbec_sdk -name '*.so*' -exec cp {} /usr/local/lib/ \; \
    && ldconfig \
    && rm /tmp/orbbec_sdk.zip

RUN find /opt/orbbec_sdk -name '99-obsensor-libusb.rules' \
    -exec cp {} /etc/udev/rules.d/ \; 2>/dev/null || true

# ─── PyTorch for Jetson (CUDA, YOLO) ────────────────────────────────────────
RUN pip3 install --no-cache-dir \
    torch torchvision torchaudio \
    --index-url https://pypi.jetson-platform.net/simple/ \
    || echo "WARN: Jetson PyTorch index failed — torch may already be installed"

# ─── Python packages ────────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir \
    ultralytics \
    scipy \
    transforms3d \
    numpy \
    opencv-python-headless

# ─── rosdep init ────────────────────────────────────────────────────────────
RUN rosdep init 2>/dev/null || true && rosdep update

# ─── CycloneDDS RMW + rosidl + diagnostic (non in ros_base) ─────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-default-generators \
    ros-humble-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

# ─── Workspace (mounted as volume at runtime) ───────────────────────────────
WORKDIR /ros2_ws

# ─── Workspace source + build (immagine self-contained) ──────────────────
COPY src/ /ros2_ws/src/
RUN . /opt/ros/humble/setup.bash && \
    . /opt/ros/humble/install/setup.bash 2>/dev/null || true && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /ros2_ws/build /ros2_ws/log

# ─── Entrypoint ─────────────────────────────────────────────────────────────
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
