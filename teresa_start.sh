#!/bin/bash
# ============================================================
# TERESA — Avvia i container Docker (solo container, niente launch)
# Uso:  bash teresa_start.sh            (avvia entrambi)
#       bash teresa_start.sh stop       (ferma tutto)
#       bash teresa_start.sh status     (stato container)
# ============================================================
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
DOMAIN=42
WS_MOUNT="${SCRIPT_DIR}:/ros2_ws"

case "${1:-start}" in
  stop)
    echo "[TERESA] Fermo container..."
    docker rm -f teresa_core teresa_gpu 2>/dev/null
    echo "[TERESA] Fermato."
    ;;
  status)
    echo "[TERESA] Stato:"
    docker ps --filter name=teresa --format '  {{.Names}}  {{.Status}}  {{.Image}}' 2>/dev/null || echo "  nessun container attivo"
    ;;
  start|*)
    echo "[TERESA] Avvio container..."

    # Ferma precedenti
    docker rm -f teresa_core teresa_gpu 2>/dev/null

    # T1 — Hardware container (teresa_core: camere + Z1 + TF)
    echo "[TERESA] teresa_core (hardware)..."
    docker run -d --name teresa_core --rm --net=host --privileged       -e ROS_DOMAIN_ID=$DOMAIN       -v $WS_MOUNT       teresa_core:latest sleep infinity

    # T2 — GPU perception container
    echo "[TERESA] teresa_gpu (percezione)..."
    docker run -d --name teresa_gpu --rm --runtime=nvidia --net=host       -e ROS_DOMAIN_ID=$DOMAIN       -v $WS_MOUNT       teresa_gpu:latest sleep infinity

    echo "[TERESA] Pronto!"
    echo ""
    echo "  Entra core:  docker exec -it teresa_core bash"
    echo "  Entra GPU:     docker exec -it teresa_gpu bash"
    echo "  Ferma tutto:   bash teresa_start.sh stop"
    ;;
esac
