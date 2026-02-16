# ============================================================
# MULTI-STAGE BUILD: RealSense + Spot ROS2 + YOLO11
# ============================================================

ARG BASE_IMAGE=ubuntu:22.04
ARG LIBRS_VERSION=2.56.5

#################################
#   Librealsense Builder Stage  #
#################################
FROM $BASE_IMAGE as librealsense-builder

ARG LIBRS_VERSION
# Make sure that we have a version number of librealsense as argument
RUN test -n "$LIBRS_VERSION"

# To avoid waiting for input during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Builder dependencies installation
RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \    
    curl \
    python3 \
    python3-dev \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Download sources
WORKDIR /usr/src
RUN curl https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v$LIBRS_VERSION -o librealsense.tar.gz 
RUN tar -zxf librealsense.tar.gz \
    && rm librealsense.tar.gz 
RUN ln -s /usr/src/librealsense-$LIBRS_VERSION /usr/src/librealsense

# Build and install
RUN cd /usr/src/librealsense \
    && mkdir build && cd build \
    && cmake \
    -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
    -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
    -DCMAKE_INSTALL_PREFIX=/opt/librealsense \    
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_PYTHON_BINDINGS:bool=true \
    -DCMAKE_BUILD_TYPE=Release ../ \
    && make -j$(($(nproc)-1)) all \
    && make install 

######################################
#   FINAL STAGE: Spot + RealSense   #
######################################
FROM ${BASE_IMAGE} as final

#RMW ZENOH experimental flag
ARG EXPERIMENTAL_ZENOH_RMW=FALSE

# Set noninteractive mode for APT
ENV DEBIAN_FRONTEND=noninteractive

# Env setup
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

# ============================================================
# COPY REALSENSE FROM BUILDER STAGE
# ============================================================
COPY --from=librealsense-builder /opt/librealsense /usr/local/
COPY --from=librealsense-builder /usr/lib/python3/dist-packages/pyrealsense2 /usr/lib/python3/dist-packages/pyrealsense2
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib

# ============================================================
# BASE DEPENDENCIES
# ============================================================
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    git \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    libusb-1.0-0 \
    udev \
    apt-transport-https \
    ca-certificates \
    software-properties-common \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# ROS 2 HUMBLE INSTALLATION
# ============================================================
# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update

# Install ROS2 and dependencies
RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
    wget \
    software-properties-common \
    python3-pip \
    python-is-python3 \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    libpython3-dev \
    python3-tk \
    qttools5-dev \
    ros-humble-ros-base \
    ros-dev-tools \
    ros-humble-teleop-twist-keyboard \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-diagnostic-updater \
    #check if Zenoh should be installed
    $(if [ "$EXPERIMENTAL_ZENOH_RMW" = "TRUE" ]; then echo "ros-humble-rmw-zenoh-cpp"; fi) \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# SET UP ROS2 WORKSPACE
# ============================================================
WORKDIR /ros_ws/src

# Initialize rosdep
RUN rosdep init && rosdep update

# ============================================================
# SPOT ROS2 DRIVER
# ============================================================
# Clone Spot driver code
RUN git clone https://github.com/bdaiinstitute/spot_ros2.git spot_ros2
WORKDIR /ros_ws/src/spot_ros2
RUN git submodule update --init --recursive

# Run install script
RUN ARCH=$(dpkg --print-architecture) && echo "Building driver with $ARCH" && /ros_ws/src/spot_ros2/install_spot_ros2.sh --$ARCH

# ============================================================
# REALSENSE ROS2 WRAPPER
# ============================================================
WORKDIR /ros_ws/src
RUN git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master

# ============================================================
# COPY YOUR LOCAL CODE
# ============================================================
COPY src/my_spot_controller /ros_ws/src/my_spot_controller

# ============================================================
# YOLO11 INSTALLATION
# ============================================================
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel
RUN pip3 install --no-cache-dir torch>=2.0.0 torchvision>=0.15.0
RUN pip3 install --no-cache-dir opencv-python-headless
RUN pip3 install --no-cache-dir "ultralytics>=8.3.0"

# Verify YOLO11 installation
RUN python3 -c "\
from ultralytics import YOLO; \
import numpy as np; \
import cv2; \
import torch; \
print('=' * 60); \
print('🔍 Verifying YOLO11 installation...'); \
print('✅ OpenCV:', cv2.__version__); \
print('✅ PyTorch:', torch.__version__, '| CUDA:', torch.cuda.is_available()); \
print('🚀 Downloading YOLO11n-pose model...'); \
model = YOLO('yolo11n-pose.pt'); \
dummy = np.zeros((640, 640, 3), dtype=np.uint8); \
results = model(dummy, verbose=False); \
print('✅ Model inference test passed'); \
print('✅ YOLO11 installation complete!'); \
print('=' * 60); \
"

RUN pip3 install --upgrade "setuptools>=58.0.0,<70.0.0"

# ============================================================
# BUILD ALL ROS PACKAGES TOGETHER
# ============================================================
WORKDIR /ros_ws/
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# ============================================================
# SETUP ENTRYPOINT
# ============================================================
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros_ws/install/setup.bash" >> ~/.bashrc && \
    echo "echo '✅ Spot + RealSense + YOLO11 ready'" >> ~/.bashrc

CMD ["bash"]
