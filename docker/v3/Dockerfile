FROM dustynv/ros:humble-ros-base-l4t-r35.4.1

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    libusb-1.0-0-dev \
    libudev-dev \
    udev \
    cmake \
    build-essential \
    git \
    wget \
    ca-certificates \
    libgflags-dev \
    nlohmann-json3-dev \
    libgoogle-glog-dev \
    && rm -rf /var/lib/apt/lists/*

ARG REALSENSE_VERSION=v2.57.7
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
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

ARG ORBBEC_SDK_VERSION=v2.0.5
RUN wget -q \
    https://github.com/orbbec/OrbbecSDK/releases/download/${ORBBEC_SDK_VERSION}/OrbbecSDK_${ORBBEC_SDK_VERSION}_linux_arm64.tar.gz \
    -O /tmp/orbbec_sdk.tar.gz && \
    mkdir -p /opt/orbbec_sdk && \
    tar -xzf /tmp/orbbec_sdk.tar.gz -C /opt/orbbec_sdk --strip-components=1 && \
    cp /opt/orbbec_sdk/lib/*.so* /usr/local/lib/ && \
    ldconfig && \
    rm /tmp/orbbec_sdk.tar.gz

RUN if [ -f /opt/orbbec_sdk/misc/99-obsensor-libusb.rules ]; then \
        cp /opt/orbbec_sdk/misc/99-obsensor-libusb.rules /etc/udev/rules.d/; \
    fi

RUN pip3 install --no-cache-dir \
    ultralytics==8.4.50 \
    pin \
    scipy>=1.10.0 \
    numpy>=1.24.0 \
    transforms3d>=0.4.1 \
    tf_transformations \
    opencv-python-headless>=4.8.0

RUN rosdep init 2>/dev/null || true && rosdep update

ARG ROS_DISTRO=humble
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    mkdir -p /tmp/ros2_extra/src && \
    cd /tmp/ros2_extra && \
    git clone --depth 1 -b ${ROS_DISTRO} \
        https://github.com/ros/xacro.git src/xacro && \
    git clone --depth 1 -b ${ROS_DISTRO} \
        https://github.com/ros/robot_state_publisher.git src/robot_state_publisher && \
    colcon build --merge-install --install-base /opt/ros/${ROS_DISTRO} \
        --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /tmp/ros2_extra

WORKDIR /ros2_ws

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
