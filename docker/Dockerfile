# ─────────────────────────────────────────────────────────────────────────────
# TERESA Jetson Docker
# Base: dustynv ROS2 Humble + CUDA per JetPack 5.x (L4T r35)
#
# VERIFICA la versione L4T sulla Jetson prima di buildare:
#   cat /etc/nv_tegra_release
# Poi aggiorna il tag di conseguenza:
#   r35.1.0 → JP 5.0.2
#   r35.2.1 → JP 5.1
#   r35.3.1 → JP 5.1.1
#   r35.4.1 → JP 5.1.2  (default qui)
#   r35.5.0 → JP 5.1.3
# ─────────────────────────────────────────────────────────────────────────────
FROM dustynv/ros:humble-ros-base-l4t-r35.4.1

# ─── System deps ─────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # USB / device access
    libusb-1.0-0-dev \
    libudev-dev \
    udev \
    # build tools
    cmake \
    build-essential \
    git \
    wget \
    ca-certificates \
    # ROS2 deps comuni
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-vision-msgs \
    ros-humble-visualization-msgs \
    ros-humble-ament-cmake \
    ros-humble-rosidl-default-generators \
    ros-humble-action-msgs \
    ros-humble-control-msgs \
    ros-humble-trajectory-msgs \
    # spot_msgs (se non presente su SpotCore, build from source)
    && rm -rf /var/lib/apt/lists/*

# ─── librealsense2 per Jetson (RSUSB backend) ────────────────────────────────
# Intel non fornisce binari apt ufficiali per ARM/Jetson → build from source.
# Tag: verifica ultima release su github.com/IntelRealSense/librealsense
ARG REALSENSE_VERSION=v2.55.1
RUN apt-get update && apt-get install -y --no-install-recommends \
    libssl-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch ${REALSENSE_VERSION} \
        https://github.com/IntelRealSense/librealsense.git /tmp/librealsense && \
    cd /tmp/librealsense && \
    mkdir build && cd build && \
    cmake .. \
        -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_PYTHON_BINDINGS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/librealsense

# ─── Orbbec SDK per Femto Bolt (ARM64) ───────────────────────────────────────
# Scarica OrbbecSDK v2 per Linux ARM64.
# Verifica ultima versione su: github.com/orbbec/OrbbecSDK/releases
ARG ORBBEC_SDK_VERSION=v2.0.5
RUN wget -q \
    https://github.com/orbbec/OrbbecSDK/releases/download/${ORBBEC_SDK_VERSION}/OrbbecSDK_${ORBBEC_SDK_VERSION}_linux_arm64.tar.gz \
    -O /tmp/orbbec_sdk.tar.gz && \
    mkdir -p /opt/orbbec_sdk && \
    tar -xzf /tmp/orbbec_sdk.tar.gz -C /opt/orbbec_sdk --strip-components=1 && \
    cp /opt/orbbec_sdk/lib/*.so* /usr/local/lib/ && \
    ldconfig && \
    rm /tmp/orbbec_sdk.tar.gz

# Regole udev Orbbec (copiate nell'immagine, applicate dall'host tramite run.sh)
RUN if [ -f /opt/orbbec_sdk/misc/99-obsensor-libusb.rules ]; then \
        cp /opt/orbbec_sdk/misc/99-obsensor-libusb.rules /etc/udev/rules.d/; \
    fi

# ─── Python deps ─────────────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir \
    # YOLO
    ultralytics \
    # Pinocchio (IK per Z1)
    pin \
    # Utility
    scipy \
    numpy \
    transforms3d \
    # tf_transformations (ROS2)
    tf_transformations \
    # OpenCV headless (dustynv ha già opencv ma per sicurezza)
    opencv-python-headless

# ─── rosdep init ─────────────────────────────────────────────────────────────
RUN rosdep init 2>/dev/null || true && rosdep update

# ─── Workspace ───────────────────────────────────────────────────────────────
# Il workspace viene montato come volume a runtime (-v /path/TERESA_ws:/ros2_ws).
# Non viene copiato nell'immagine: git pull sull'host → rebuild dentro il container.
WORKDIR /ros2_ws

# ─── Entrypoint ──────────────────────────────────────────────────────────────
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
