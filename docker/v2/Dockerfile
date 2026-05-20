# ─────────────────────────────────────────────────────────────────────────────
# TERESA Jetson Docker — v2
# Base: dustynv ROS2 Humble + CUDA per JetPack 5.x (L4T r35)
#
# IMPORTANTE: le immagini dustynv buildano ROS2 da sorgente.
# I pacchetti ros-humble-* NON vanno installati da apt — non esistono.
# Quelli core (tf2, cv_bridge, image_transport, xacro, robot_state_publisher...)
# sono già inclusi in ros_base. Solo pinocchio va aggiunto via /ros2_install.sh.
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

# ─── Fix ROS2 GPG key (expired) ──────────────────────────────────────────────
# La chiave serve a rosdep per risolvere le dipendenze di sistema (non i
# pacchetti ROS). Seguiamo lo stesso pattern di dustynv/ros2_build.sh.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# ─── System deps ─────────────────────────────────────────────────────────────
# Solo pacchetti NON-ROS. I pacchetti ROS2 core sono già nel base image.
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
    # orbbec_camera build deps
    libgflags-dev \
    nlohmann-json3-dev \
    libgoogle-glog-dev \
    && rm -rf /var/lib/apt/lists/*

# ─── librealsense2 per Jetson (RSUSB backend) ────────────────────────────────
# Intel non fornisce binari apt ufficiali per ARM/Jetson → build from source.
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
# La naming convention è cambiata: ora .zip con hash commit nel nome.
# Ultima release testata: v1.10.27
ARG ORBBEC_SDK_VERSION=v1.10.27
RUN apt-get update && apt-get install -y --no-install-recommends unzip \
    && rm -rf /var/lib/apt/lists/* \
    && wget -q \
    "https://github.com/orbbec/OrbbecSDK/releases/download/${ORBBEC_SDK_VERSION}/OrbbecSDK_C_C%2B%2B_${ORBBEC_SDK_VERSION}_20250925_0549823_linux_arm64_release.zip" \
    -O /tmp/orbbec_sdk.zip \
    && mkdir -p /opt/orbbec_sdk \
    && unzip -q /tmp/orbbec_sdk.zip -d /opt/orbbec_sdk \
    && cp /opt/orbbec_sdk/*/lib/*.so* /usr/local/lib/ 2>/dev/null \
    || cp /opt/orbbec_sdk/lib/*.so* /usr/local/lib/ \
    && ldconfig \
    && rm /tmp/orbbec_sdk.zip

# Regole udev Orbbec (cercate ricorsivamente, path cambiato nelle nuove release)
RUN find /opt/orbbec_sdk -name '99-obsensor-libusb.rules' -exec cp {} /etc/udev/rules.d/ \; 2>/dev/null || true

# ─── Python deps ─────────────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir \
    ultralytics==8.4.50 \
    scipy>=1.10.0 \
    numpy>=1.24.0 \
    transforms3d>=0.4.1 \
    opencv-python-headless>=4.8.0

# ─── Pinocchio (build da sorgente via /ros2_install.sh) ──────────────────────
# Pinocchio NON è incluso in ros_base — va buildato da sorgente.
# /ros2_install.sh scarica le dipendenze, risolve rosdep e compila con colcon.
RUN /ros2_install.sh pinocchio

# ─── rosdep init ─────────────────────────────────────────────────────────────
RUN rosdep init 2>/dev/null || true && rosdep update

# ─── Workspace ───────────────────────────────────────────────────────────────
# Il workspace viene montato come volume a runtime (-v /path/TERESA_ws:/ros2_ws).
WORKDIR /ros2_ws

# ─── Entrypoint ──────────────────────────────────────────────────────────────
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
