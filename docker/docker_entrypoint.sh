#!/bin/bash
# ============================================================================
# TERESA — Docker Entrypoint for Jetson Orin AGX (L4T R35.x)
#
# What this does:
# 1. Sources ROS2 Humble environment
# 2. Sources TERESA workspace (if built)
# 3. Prints useful info (GPU status, ROS2 topics, workspace packages)
# 4. Executes CMD (default: bash)
# ============================================================================

set -e

echo "=============================================="
echo "  TERESA_ws — Jetson Orin AGX Container"
echo "  ROS2 Humble | L4T R35.x | CUDA 11.4"
echo "=============================================="

# ── Source ROS2 ──────────────────────────────────────────────────────
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "[OK] ROS2 Humble sourced from /opt/ros/humble"
else
    echo "[WARN] /opt/ros/humble/setup.bash not found"
fi

# ── Source TERESA workspace ──────────────────────────────────────────
if [ -f /root/TERESA_ws/install/setup.bash ]; then
    source /root/TERESA_ws/install/setup.bash
    echo "[OK] TERESA workspace sourced from /root/TERESA_ws/install"
else
    echo "[INFO] TERESA workspace not built yet. Run: colcon build"
fi

# ── Check GPU ────────────────────────────────────────────────────────
if python3 -c "import torch; print(torch.cuda.is_available())" 2>/dev/null | grep -q True; then
    GPU_NAME=$(python3 -c "import torch; print(torch.cuda.get_device_name(0))" 2>/dev/null || echo "Unknown")
    echo "[OK] GPU detected: $GPU_NAME"
else
    echo "[WARN] CUDA GPU not available. Ensure --runtime=nvidia is set."
fi

# ── Check models ─────────────────────────────────────────────────────
if [ -f /root/TERESA_ws/yolo11n-pose.pt ]; then
    echo "[OK] YOLO model: yolo11n-pose.pt"
else
    echo "[WARN] YOLO model not found. Will auto-download on first use."
fi

if [ -f /root/TERESA_ws/nlf_s_multi.torchscript ]; then
    echo "[OK] NLF model: nlf_s_multi.torchscript"
else
    echo "[WARN] NLF model not found. Run: bash scripts/download_nlf_models.sh"
fi

# ── Print ROS2 environment ───────────────────────────────────────────
echo ""
echo "Environment:"
echo "  ROS_DISTRO:     ${ROS_DISTRO:-humble}"
echo "  ROS_DOMAIN_ID:  ${ROS_DOMAIN_ID:-0}"
echo "  RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
echo ""

# ── Run CMD ──────────────────────────────────────────────────────────
exec "$@"
