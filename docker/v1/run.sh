#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# Avvia il container TERESA sulla Jetson.
# Usa: ./run.sh [comando opzionale]
# Es:  ./run.sh                          → bash interattivo
#      ./run.sh ros2 launch z1_vision z1_control.launch.py
# ─────────────────────────────────────────────────────────────────────────────

WORKSPACE_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
IMAGE_NAME="teresa_ws:v1"
CONTAINER_DIR="$(dirname "$0")"

# Applica regole udev Orbbec sull'host (solo la prima volta)
if [ -f /etc/udev/rules.d/99-obsensor-libusb.rules ]; then
    echo "[run.sh] udev rules Orbbec già presenti."
else
    echo "[run.sh] Copio udev rules Orbbec sull'host..."
    sudo cp "${CONTAINER_DIR}/99-obsensor-libusb.rules" /etc/udev/rules.d/ 2>/dev/null || true
    sudo udevadm control --reload-rules && sudo udevadm trigger || true
fi

docker run -it --rm \
    --runtime nvidia \
    --network host \
    --privileged \
    --ipc host \
    -e DISPLAY="${DISPLAY}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "${WORKSPACE_DIR}:/ros2_ws" \
    -v /dev:/dev \
    -v /run/udev:/run/udev:ro \
    "${IMAGE_NAME}" \
    "$@"
