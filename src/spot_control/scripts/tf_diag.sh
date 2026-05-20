#!/usr/bin/env bash
set -euo pipefail

echo "========================================"
echo " TERESA TF Diagnostic"
echo "========================================"
echo ""

# --- 1. DDS connectivity check ---
echo "[1] DDS — topic list (cerca /tf e /tf_static)"
echo "----------------------------------------"
TF_TOPICS=$(ros2 topic list 2>/dev/null | grep -E '/(tf|tf_static)' || true)
if [ -z "$TF_TOPICS" ]; then
    echo "  NESSUN topic /tf o /tf_static trovato!"
    echo "  => DDS non funziona tra PC e SpotCore."
    echo "  Verifica:"
    echo "    - spot_ros2 attivo su SpotCore?"
    echo "    - ROS_DOMAIN_ID uguale su entrambe le macchine?"
    echo "      echo \$ROS_DOMAIN_ID"
    echo "    - Ping reciproco funziona?"
    echo "      ping <IP_SpotCore>"
    echo "    - Prova con CycloneDDS:"
    echo "      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
else
    echo "  OK — topic TF trovati:"
    echo "$TF_TOPICS"
fi

echo ""

# --- 2. Frame list ---
echo "[2] TF — frame attivi (da tf2_monitor, 3 secondi)"
echo "----------------------------------------"
timeout 3 ros2 run tf2_ros tf2_monitor 2>/dev/null || echo "  tf2_monitor non ha prodotto output (DDS ok?)"
echo ""

# --- 3. spot_name ---
echo "[3] spot_name dal driver Spot"
echo "----------------------------------------"
SPOT_NAME=$(ros2 param get /spot_driver spot_name 2>/dev/null || echo "")
if [ -z "$SPOT_NAME" ]; then
    echo "  impossibile leggere spot_name da /spot_driver."
    echo "  Il nodo spot_driver è attivo? Controlla con:"
    echo "    ros2 node list | grep spot"
    echo "  Se assente, il namespace di default è 'my_spot'."
else
    echo "  spot_name = $SPOT_NAME"
fi

echo ""

# --- 4. Node list ---
echo "[4] Nodi ROS 2 visibili"
echo "----------------------------------------"
ros2 node list 2>/dev/null | head -30 || echo "  nessun nodo visibile"
echo ""

# --- 5. Specific frame check ---
echo "[5] Frame critici — my_spot/odom e my_spot/body"
echo "----------------------------------------"
echo "  (3 secondi per far arrivare i TF...)"
timeout 3 ros2 run tf2_ros tf2_echo my_spot/odom my_spot/body 2>/dev/null || echo "  tf2_echo fallito — i frame non sono disponibili"
echo ""

echo "========================================"
echo " Diagnostica completata"
echo "========================================"
