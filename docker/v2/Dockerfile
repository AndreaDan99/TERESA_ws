# ─────────────────────────────────────────────────────────────────────────────
# TERESA Jetson Docker — v2
# Base: dustynv ROS2 Humble + CUDA per JetPack 5.x (L4T r35)
#
# Differenze rispetto a v1:
#   - Pinocchio installato via apt (ros-humble-pinocchio), non pip "pin"
#   - tf_transformations via apt (ros-humble-tf-transformations), non pip
#   - Aggiunte dipendenze apt mancanti (message_filters, gflags, glog, nlohmann-json)
#   - librealsense v2.57.7 (era v2.55.1)
#   - Versioni pip pinnate per riproducibilità
#   - File udev Orbbec incluso nella cartella v2/
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

# ─── Fix ROS2 GPG key (expired) + apt source (jammy for Humble) ──────────────
# Base image is Focal-based (L4T r35) but ros-humble packages are only on the
# jammy pool at packages.ros.org. Rewrite ros2.list accordingly.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

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
    ros-humble-tf-transformations \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-vision-msgs \
    ros-humble-visualization-msgs \
    ros-humble-ament-cmake \
    ros-humble-rosidl-default-generators \
    ros-humble-action-msgs \
    ros-humble-control-msgs \
    ros-humble-trajectory-msgs \
    ros-humble-message-filters \
    ros-humble-pinocchio \
    # orbbec_camera build deps
    libgflags-dev \
    nlohmann-json3-dev \
    libgoogle-glog-dev \
    # xacro + robot_state_publisher (runtime, z1_bringup)
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# ─── librealsense2 per Jetson (RSUSB backend) ────────────────────────────────
# Intel non fornisce binari apt ufficiali per ARM/Jetson → build from source.
# Tag: verifica ultima release su github.com/IntelRealSense/librealsense
ARG REALSENSE_VERSION=v2.57.7
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
# Pinocchio e tf_transformations sono installati via apt sopra.
RUN pip3 install --no-cache-dir \
    ultralytics==8.4.50 \
    scipy>=1.10.0 \
    numpy>=1.24.0 \
    transforms3d>=0.4.1 \
    opencv-python-headless>=4.8.0

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
