#!/bin/bash
# ============================================================
#  TERESA container entrypoint (v2)
#  Sources ROS2 + workspace, then executes the given command.
# ============================================================
set -e

source /opt/ros/humble/setup.bash

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
