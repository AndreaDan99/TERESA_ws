#!/bin/bash
# ============================================================
#  TERESA container entrypoint (v2)
#  Sources ROS2 + workspace, then executes the given command.
# ============================================================
set -e

# Ensure system Python 3.10 (not venv's 3.12) is used by cmake/colcon
export PATH=/usr/bin:$PATH
export COLCON_DEFAULTS_FILE=/ros2_ws/colcon_defaults.yaml 2>/dev/null || true

cat << 'YAML' > /colcon_defaults.yaml
build:
  cmake-args:
    - -DPython3_EXECUTABLE=/usr/bin/python3
    - -DPython3_NumPy_INCLUDE_DIRS=/usr/lib/python3/dist-packages/numpy/core/include
YAML

source /opt/ros/humble/setup.bash
# dustynv base image installs ROS2 under .../install/ (merge-install layout)
if [ -f /opt/ros/humble/install/setup.bash ]; then
    source /opt/ros/humble/install/setup.bash
fi

# Fix: dustynv base image's venv has a broken cmake that shadows system cmake
for d in /opt/venv/lib/python*/site-packages/cmake; do
    rm -rf "$d" 2>/dev/null || true
done
hash -r 2>/dev/null || true

# Fix: base image's builtin_interfaces expects numpy headers at old path
python3 -c "
import numpy, os
expected = '/usr/local/lib/python3.10/dist-packages/numpy/core/include'
if not os.path.exists(expected):
    actual = numpy.get_include()
    if os.path.exists(actual):
        os.makedirs(os.path.dirname(expected), exist_ok=True)
        os.symlink(actual, expected)
        print(f'numpy include symlink: {expected} -> {actual}')
" 2>/dev/null || true

if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
else
    echo "INFO: /ros2_ws not built yet — run 'colcon build' first"
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}

chmod -R a+rw /dev/bus/usb 2>/dev/null || true

echo "╔══════════════════════════════════════╗"
echo "║  TERESA — ROS2 Humble                ║"
echo "║  Domain ID: ${ROS_DOMAIN_ID}                         ║"
echo "╚══════════════════════════════════════╝"
echo ""
echo "Quick launch:"
echo "  Z1 standalone:"
echo "    ros2 launch z1_vision z1_realsense.launch.py use_rviz:=true"
echo "    ros2 launch z1_vision z1_perception.launch.py"
echo "    ros2 launch z1_vision z1_control.launch.py"
echo ""
echo "  Full system (SpotCore running):"
echo "    ros2 launch spot_perception spot_perception.launch.py"
echo "    ros2 launch z1_vision z1_realsense.launch.py use_rviz:=true"
echo "    ros2 launch z1_vision z1_perception.launch.py"
echo "    ros2 launch z1_vision z1_control.launch.py"
echo "    ros2 launch spot_control wbc.launch.py"
echo ""

exec "$@"
