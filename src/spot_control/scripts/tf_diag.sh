#!/usr/bin/env bash
# TERESA TF Diagnostic — verifica tutte le catene TF
# Uso:  bash src/spot_control/scripts/tf_diag.sh
set -euo pipefail

RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[0;33m'
NC='\033[0m'

PASS=0
FAIL=0
CHECK_NUM=0

section() { echo -e "\n${1}"; }
ok()      { PASS=$((PASS+1)); echo -e "  ${GRN}OK${NC}        $1"; }
fail()    { FAIL=$((FAIL+1)); echo -e "  ${YLW}MANCANTE${NC}  $1"; }

check_frame() {
    # $1 = source frame, $2 = target frame, $3 = type (static|dynamic)
    # $4 = chi lo fornisce, $5 = launch file
    local SRC="$1" TGT="$2" TYPE="$3" PROVIDER="$4" LAUNCH="$5"
    CHECK_NUM=$((CHECK_NUM + 1))

    # Run tf2_echo for 2 seconds, capture any transform line
    local OUT
    OUT=$(timeout 2 ros2 run tf2_ros tf2_echo "$SRC" "$TGT" 2>&1 || true)

    if echo "$OUT" | grep -q "Translation"; then
        ok "[$CHECK_NUM] $TYPE  $SRC → $TGT  ($PROVIDER)"
    else
        fail "[$CHECK_NUM] $TYPE  $SRC → $TGT  ($PROVIDER)"
        echo -e "          Avvia: ${YLW}ros2 launch${NC} $LAUNCH"
    fi
}

# ═══════════════════════════════════════════════════════════════════════
echo "════════════════════════════════════════"
echo "  TERESA TF Diagnostic"
echo "════════════════════════════════════════"
echo ""

# ── 0. ros2 disponibile? ─────────────────────────────────────────────
if ! command -v ros2 &>/dev/null; then
    echo -e "${RED}FATAL${NC}: ros2 non trovato nel PATH. Fai 'source install/setup.bash'?"
    exit 1
fi

# ── DDS connectivity ──────────────────────────────────────────────────
section "── DDS connectivity ──────────────────────────────"
TF_TOPICS=$(ros2 topic list 2>/dev/null | grep -E '/(tf|tf_static)' || true)
if [ -z "$TF_TOPICS" ]; then
    fail "DDS — topic /tf o /tf_static NON visibili"
    echo -e "  SpotCore NON raggiungibile via DDS."
    echo -e "  Verifica: ${YLW}ROS_DOMAIN_ID${NC} uguale?  ping <SpotCore IP>?"
    echo -e "  I check TF puntuali verranno comunque eseguiti..."
    echo ""
else
    ok "DDS — topic TF trovati: $TF_TOPICS"
fi

# ── Check puntuali ────────────────────────────────────────────────────
section "── TF tree (7 catene) ────────────────────────────"

# Catena SpotCore → body (DINAMICA, SpotCore)
check_frame "my_spot/odom"       "my_spot/body" \
    "dynamic" "spot_ros2 su SpotCore (DDS)" \
    "(spot_ros2 già attivo su SpotCore)"

# Catena Orbbec (STATICA, spot_perception)
check_frame "my_spot/body"       "orbbec_link" \
    "static"  "static_transform_publisher" \
    "spot_perception spot_perception.launch.py"

check_frame "orbbec_link"        "orbbec_color_optical_frame" \
    "static"  "static_transform_publisher" \
    "spot_perception spot_perception.launch.py"

# Montaggio Z1 su Spot (STATICA, wbc_qp_controller dentro wbc.launch)
check_frame "my_spot/body"       "link00" \
    "static"  "wbc_qp_controller (StaticTransformBroadcaster)" \
    "spot_control wbc.launch.py"

# Catena cinematica Z1 (DINAMICA, robot_state_publisher da joint_states)
check_frame "link00"             "link06" \
    "dynamic" "robot_state_publisher (da /joint_states)" \
    "z1_vision z1_realsense.launch.py"

# RealSense su Z1 (STATICA, camera_tf)
check_frame "link06"             "camera_link" \
    "static"  "static_transform_publisher" \
    "z1_vision z1_realsense.launch.py (camera_tf)"

check_frame "camera_link"        "camera_color_optical_frame" \
    "static"  "realsense2_camera driver" \
    "z1_vision z1_realsense.launch.py"

# ── Riepilogo ─────────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════"
echo -e "  Riepilogo: ${GRN}${PASS} OK${NC}  /  ${YLW}${FAIL} mancanti${NC}"
echo "════════════════════════════════════════"

if [ "$FAIL" -eq 0 ]; then
    echo -e "\n${GRN}✔ TF tree completo${NC} — puoi avviare i nodi TERESA."
else
    echo ""
    echo "Azioni correttive:"
    echo "  - SpotCore:     verifica spot_ros2 attivo, DDS, DOMAIN_ID"
    echo "  - Orbbec TF:    ros2 launch spot_perception spot_perception.launch.py"
    echo "  - Z1 mount TF:  ros2 launch spot_control wbc.launch.py (wbc_qp_controller)"
    echo "  - Z1 arm TF:    ros2 launch z1_vision z1_realsense.launch.py (robot_state_publisher)"
    echo "  - Camera TF:    inclusa in z1_realsense.launch.py (camera_tf + realsense driver)"
fi
echo ""
