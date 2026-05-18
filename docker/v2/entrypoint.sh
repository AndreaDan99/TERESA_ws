#!/bin/bash
set -e

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace installato (se già buildato)
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"
